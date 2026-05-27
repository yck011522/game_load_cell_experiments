"""
Experiment 2: Dual load-cell speed test using pymodbus (no manual RTU framing).

Cycle behavior:
1) Read sensor 1 real-time weight (0001H, 32-bit signed across 2 registers)
2) Read sensor 2 real-time weight (0001H, 32-bit signed across 2 registers)
3) Convert both to grams, apply startup auto-zero tare offsets, and compute:
   total_weight_1_2 = grams1 + grams2

Reports cycle rate (Hz) and per-sensor poll/update metrics.

Dependency:
    pip install pymodbus pyserial
"""

from __future__ import annotations

import argparse
import csv
import inspect
import struct
import time
from datetime import datetime
from pathlib import Path

from pymodbus.client import ModbusSerialClient


def regs_to_i32(hi_u16: int, lo_u16: int) -> int:
    combined = (hi_u16 << 16) | lo_u16
    return struct.unpack(">i", struct.pack(">I", combined))[0]


def raw_to_grams(raw_scaled: float, zero_count: float, grams_per_count: float) -> float:
    return (raw_scaled - zero_count) * grams_per_count


def read_holding_registers_compat(
    client: ModbusSerialClient, address: int, count: int, slave: int
):
    """Call read_holding_registers across pymodbus API variants."""
    fn = client.read_holding_registers
    params = inspect.signature(fn).parameters

    if "slave" in params:
        return fn(address=address, count=count, slave=slave)
    if "unit" in params:
        return fn(address=address, count=count, unit=slave)
    if "device_id" in params:
        return fn(address=address, count=count, device_id=slave)

    # Fallback for older positional APIs.
    return fn(address, count, slave)


def read_decimal_places(client: ModbusSerialClient, slave: int) -> int:
    rr = read_holding_registers_compat(client, address=0x0000, count=1, slave=slave)
    if rr.isError():
        raise RuntimeError(f"Failed reading decimal places from slave {slave}: {rr}")
    return int(rr.registers[0])


def read_realtime_weight_i32(client: ModbusSerialClient, slave: int) -> int:
    # 0001H is 32-bit signed, so read 2 registers starting at 0x0001.
    rr = read_holding_registers_compat(client, address=0x0001, count=2, slave=slave)
    if rr.isError():
        raise RuntimeError(f"Failed reading real-time weight from slave {slave}: {rr}")
    hi, lo = rr.registers[0], rr.registers[1]
    return regs_to_i32(hi, lo)


def read_sensor_grams(
    client: ModbusSerialClient,
    slave: int,
    decimals: int,
    retries: int,
    retry_delay: float,
    zero_count: float,
    grams_per_count: float,
) -> tuple[float, int]:
    last_error: Exception | None = None
    for attempt in range(retries + 1):
        try:
            raw_i32 = read_realtime_weight_i32(client, slave=slave)
            scaled = raw_i32 / (10 ** decimals)
            grams = raw_to_grams(scaled, zero_count=zero_count, grams_per_count=grams_per_count)
            return grams, raw_i32
        except Exception as ex:
            last_error = ex
            if attempt < retries and retry_delay > 0:
                time.sleep(retry_delay)
    if last_error is None:
        raise RuntimeError("Read failed for unknown reason")
    raise last_error


def main() -> None:
    parser = argparse.ArgumentParser(description="Experiment2: dual load-cell Modbus speed test using pymodbus")
    parser.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--slaves", default="1,2", help="Comma-separated slave addresses (default: 1,2)")
    parser.add_argument("--seconds", type=float, default=10.0, help="Test duration in seconds")
    parser.add_argument("--timeout", type=float, default=0.05, help="Serial timeout in seconds")
    parser.add_argument("--retries", type=int, default=1, help="Retries after timeout per request")
    parser.add_argument("--retry-delay", type=float, default=0.0005, help="Delay between retries in seconds")
    parser.add_argument("--inter-request-delay", type=float, default=0.0, help="Delay after each request in seconds")
    parser.add_argument("--zero-count", type=float, default=0.0, help="Zero/tare count in scaled units")
    parser.add_argument("--grams-per-count", type=float, default=1.0, help="Grams per scaled count")
    parser.add_argument("--print-every", type=int, default=20, help="Print one sample every N cycles")
    parser.add_argument("--tare-samples", type=int, default=20, help="Startup cycle samples for auto-zero")
    parser.add_argument("--no-auto-zero", action="store_true", help="Disable startup auto-zero")
    parser.add_argument(
        "--output-dir",
        default="logs",
        help="Directory for CSV and summary markdown output (default: logs)",
    )
    args = parser.parse_args()

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"experiment2_{run_id}.csv"
    summary_path = output_dir / f"experiment2_summary_{run_id}.md"

    slaves = [int(x.strip()) for x in args.slaves.split(",") if x.strip()]
    if len(slaves) != 2:
        raise ValueError("This cycle mode expects exactly two slave addresses, e.g. --slaves 1,2")
    s1, s2 = slaves

    client = ModbusSerialClient(
        port=args.port,
        baudrate=args.baud,
        timeout=args.timeout,
        stopbits=1,
        bytesize=8,
        parity="N",
    )

    if not client.connect():
        raise RuntimeError(f"Failed to open serial port {args.port}")

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "cycle",
                "elapsed_s",
                "sensor1_id",
                "sensor1_grams",
                "sensor2_id",
                "sensor2_grams",
                "total_weight_1_2",
                "sensor1_raw_i32",
                "sensor2_raw_i32",
            ])

            decimals_by_slave: dict[int, int] = {}
            for slave in slaves:
                try:
                    decimals_by_slave[slave] = read_decimal_places(client, slave=slave)
                except Exception:
                    decimals_by_slave[slave] = 2

            print(f"Slave {s1} decimal places: {decimals_by_slave[s1]}")
            print(f"Slave {s2} decimal places: {decimals_by_slave[s2]}")
            print(
                "Grams conversion: scaled = raw_i32 / 10^decimal_places, "
                "grams = (scaled - zero_count) * grams_per_count "
                f"with zero_count={args.zero_count}, grams_per_count={args.grams_per_count}"
            )

            tare_grams_by_slave = {s1: 0.0, s2: 0.0}
            if not args.no_auto_zero and args.tare_samples > 0:
                print(f"Auto-zero: collecting {args.tare_samples} startup cycle samples...")
                sum_g1 = 0.0
                sum_g2 = 0.0
                tare_ok = 0
                for _ in range(args.tare_samples):
                    try:
                        g1, _ = read_sensor_grams(
                            client=client,
                            slave=s1,
                            decimals=decimals_by_slave[s1],
                            retries=args.retries,
                            retry_delay=args.retry_delay,
                            zero_count=args.zero_count,
                            grams_per_count=args.grams_per_count,
                        )
                        if args.inter_request_delay > 0:
                            time.sleep(args.inter_request_delay)
                        g2, _ = read_sensor_grams(
                            client=client,
                            slave=s2,
                            decimals=decimals_by_slave[s2],
                            retries=args.retries,
                            retry_delay=args.retry_delay,
                            zero_count=args.zero_count,
                            grams_per_count=args.grams_per_count,
                        )
                        sum_g1 += g1
                        sum_g2 += g2
                        tare_ok += 1
                        if args.inter_request_delay > 0:
                            time.sleep(args.inter_request_delay)
                    except Exception:
                        continue

                if tare_ok > 0:
                    tare_grams_by_slave[s1] = sum_g1 / tare_ok
                    tare_grams_by_slave[s2] = sum_g2 / tare_ok
                print(
                    f"Auto-zero offsets: sensor {s1}={tare_grams_by_slave[s1]:.3f} g, "
                    f"sensor {s2}={tare_grams_by_slave[s2]:.3f} g"
                )
            else:
                print("Auto-zero disabled")

            total_polls_ok = 0
            cycle_ok = 0
            err = 0
            per_slave_ok = {s1: 0, s2: 0}
            per_slave_updates = {s1: 0, s2: 0}
            last_raw_by_slave: dict[int, int] = {}
            last_total_weight_1_2: float | None = None
            total_weight_updates = 0

            t0 = time.perf_counter()
            t_end = t0 + args.seconds

            while time.perf_counter() < t_end:
                try:
                    grams1_raw, raw1_i32 = read_sensor_grams(
                        client=client,
                        slave=s1,
                        decimals=decimals_by_slave[s1],
                        retries=args.retries,
                        retry_delay=args.retry_delay,
                        zero_count=args.zero_count,
                        grams_per_count=args.grams_per_count,
                    )
                    total_polls_ok += 1
                    per_slave_ok[s1] += 1
                    grams1 = grams1_raw - tare_grams_by_slave[s1]
                    if s1 in last_raw_by_slave and last_raw_by_slave[s1] != raw1_i32:
                        per_slave_updates[s1] += 1
                    last_raw_by_slave[s1] = raw1_i32

                    if args.inter_request_delay > 0:
                        time.sleep(args.inter_request_delay)

                    grams2_raw, raw2_i32 = read_sensor_grams(
                        client=client,
                        slave=s2,
                        decimals=decimals_by_slave[s2],
                        retries=args.retries,
                        retry_delay=args.retry_delay,
                        zero_count=args.zero_count,
                        grams_per_count=args.grams_per_count,
                    )
                    total_polls_ok += 1
                    per_slave_ok[s2] += 1
                    grams2 = grams2_raw - tare_grams_by_slave[s2]
                    if s2 in last_raw_by_slave and last_raw_by_slave[s2] != raw2_i32:
                        per_slave_updates[s2] += 1
                    last_raw_by_slave[s2] = raw2_i32

                    total_weight_1_2 = grams1 + grams2
                    if last_total_weight_1_2 is not None and total_weight_1_2 != last_total_weight_1_2:
                        total_weight_updates += 1
                    last_total_weight_1_2 = total_weight_1_2

                    cycle_ok += 1
                    elapsed_now = time.perf_counter() - t0
                    csv_writer.writerow([
                        cycle_ok,
                        f"{elapsed_now:.6f}",
                        s1,
                        f"{grams1:.6f}",
                        s2,
                        f"{grams2:.6f}",
                        f"{total_weight_1_2:.6f}",
                        raw1_i32,
                        raw2_i32,
                    ])
                    if cycle_ok % args.print_every == 0:
                        print(
                            f"cycle={cycle_ok:05d} s1={s1} grams1={grams1:.3f} "
                            f"s2={s2} grams2={grams2:.3f} total_weight_1_2={total_weight_1_2:.3f}"
                        )

                    if args.inter_request_delay > 0:
                        time.sleep(args.inter_request_delay)
                except Exception as ex:
                    err += 1
                    print(f"Error #{err} in cycle read: {ex}")

            elapsed = time.perf_counter() - t0
            cycle_hz = cycle_ok / elapsed if elapsed > 0 else 0.0
            poll_hz = total_polls_ok / elapsed if elapsed > 0 else 0.0

            summary_lines = [
                "# Experiment 2 Summary",
                "",
                f"- Run ID: {run_id}",
                f"- CSV file: {csv_path}",
                f"- Elapsed: {elapsed:.3f} s",
                f"- Successful cycles: {cycle_ok}",
                f"- Successful reads (total polls): {total_polls_ok}",
                f"- Errors: {err}",
                f"- Achieved cycle rate: {cycle_hz:.2f} cycles/s",
                f"- Achieved poll rate: {poll_hz:.2f} polls/s",
                "- Target (doc max): 40.00 cycles/s for total_weight_1_2",
                f"- total_weight_1_2 value changes: {total_weight_updates} ({(total_weight_updates / elapsed) if elapsed > 0 else 0.0:.2f}/s)",
                f"- Slave {s1}: polls={per_slave_ok[s1]} ({(per_slave_ok[s1] / elapsed) if elapsed > 0 else 0.0:.2f}/s), value_changes={per_slave_updates[s1]} ({(per_slave_updates[s1] / elapsed) if elapsed > 0 else 0.0:.2f}/s)",
                f"- Slave {s2}: polls={per_slave_ok[s2]} ({(per_slave_ok[s2] / elapsed) if elapsed > 0 else 0.0:.2f}/s), value_changes={per_slave_updates[s2]} ({(per_slave_updates[s2] / elapsed) if elapsed > 0 else 0.0:.2f}/s)",
            ]

            print("\n=== Summary ===")
            for line in summary_lines[3:]:
                print(line[2:] if line.startswith("- ") else line)

            summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
            print(f"CSV written to: {csv_path}")
            print(f"Summary written to: {summary_path}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
