"""
Experiment 2: Multi-load-cell speed test using pymodbus.

Behavior:
1) Poll any number of sensor slave IDs sequentially each cycle.
2) Read real-time weight (0001H, 32-bit signed across 2 registers) per sensor.
3) Convert to grams, optionally apply startup auto-zero offsets, and print results.

Designed for fast polling with minimal overhead.

Dependency:
    pip install pymodbus pyserial
"""

from __future__ import annotations

import argparse
import inspect
import struct
import time

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
    parser = argparse.ArgumentParser(description="Experiment2: multi-load-cell Modbus speed test using pymodbus")
    parser.add_argument("--port", default="COM46", help="Serial port (default: COM12)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--slaves",
        default="1,2,3,4,5,6,7,8,9,10,11,12",
        help="Comma-separated slave addresses (example: 1,2,3,...,12)",
    )
    parser.add_argument("--seconds", type=float, default=10.0, help="Test duration in seconds")
    parser.add_argument("--timeout", type=float, default=0.05, help="Serial timeout in seconds")
    parser.add_argument("--retries", type=int, default=1, help="Retries after timeout per request")
    parser.add_argument("--retry-delay", type=float, default=0.0005, help="Delay between retries in seconds")
    parser.add_argument("--inter-request-delay", type=float, default=0.0, help="Delay after each request in seconds")
    parser.add_argument("--zero-count", type=float, default=0.0, help="Zero/tare count in scaled units")
    parser.add_argument("--grams-per-count", type=float, default=1.0, help="Grams per scaled count")
    parser.add_argument("--print-every", type=int, default=10, help="Print one sample every N cycles")
    parser.add_argument("--tare-samples", type=int, default=20, help="Startup cycle samples for auto-zero")
    parser.add_argument("--no-auto-zero", action="store_true", help="Disable startup auto-zero")
    args = parser.parse_args()

    slaves = [int(x.strip()) for x in args.slaves.split(",") if x.strip()]
    if not slaves:
        raise ValueError("At least one slave address is required, e.g. --slaves 1,2,3")

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
        decimals_by_slave: dict[int, int] = {}
        for slave in slaves:
            try:
                decimals_by_slave[slave] = read_decimal_places(client, slave=slave)
            except Exception:
                decimals_by_slave[slave] = 2

        for slave in slaves:
            print(f"Slave {slave} decimal places: {decimals_by_slave[slave]}")
        print(
            "Grams conversion: scaled = raw_i32 / 10^decimal_places, "
            "grams = (scaled - zero_count) * grams_per_count "
            f"with zero_count={args.zero_count}, grams_per_count={args.grams_per_count}"
        )

        tare_grams_by_slave = {slave: 0.0 for slave in slaves}
        if not args.no_auto_zero and args.tare_samples > 0:
            print(f"Auto-zero: collecting {args.tare_samples} startup cycle samples...")
            sums = {slave: 0.0 for slave in slaves}
            counts = {slave: 0 for slave in slaves}
            for _ in range(args.tare_samples):
                for slave in slaves:
                    try:
                        grams, _ = read_sensor_grams(
                            client=client,
                            slave=slave,
                            decimals=decimals_by_slave[slave],
                            retries=args.retries,
                            retry_delay=args.retry_delay,
                            zero_count=args.zero_count,
                            grams_per_count=args.grams_per_count,
                        )
                        sums[slave] += grams
                        counts[slave] += 1
                    except Exception:
                        pass
                    if args.inter_request_delay > 0:
                        time.sleep(args.inter_request_delay)

            for slave in slaves:
                if counts[slave] > 0:
                    tare_grams_by_slave[slave] = sums[slave] / counts[slave]
                print(f"Auto-zero offset sensor {slave}: {tare_grams_by_slave[slave]:.3f} g")
        else:
            print("Auto-zero disabled")

        total_polls_ok = 0
        cycle_ok = 0
        err = 0

        t0 = time.perf_counter()
        t_end = t0 + args.seconds

        while time.perf_counter() < t_end:
            cycle_ok += 1
            cycle_values: list[str] = []
            cycle_failed = False

            for slave in slaves:
                try:
                    grams_raw, _raw_i32 = read_sensor_grams(
                        client=client,
                        slave=slave,
                        decimals=decimals_by_slave[slave],
                        retries=args.retries,
                        retry_delay=args.retry_delay,
                        zero_count=args.zero_count,
                        grams_per_count=args.grams_per_count,
                    )
                    total_polls_ok += 1
                    grams = grams_raw - tare_grams_by_slave[slave]
                    cycle_values.append(f"s{slave}={grams:.3f}g")
                except Exception as ex:
                    cycle_failed = True
                    err += 1
                    cycle_values.append(f"s{slave}=ERR({ex})")

                if args.inter_request_delay > 0:
                    time.sleep(args.inter_request_delay)

            if args.print_every > 0 and cycle_ok % args.print_every == 0:
                status = "ERR" if cycle_failed else "OK"
                print(f"cycle={cycle_ok:06d} {status} " + " ".join(cycle_values))

        elapsed = time.perf_counter() - t0
        cycle_hz = cycle_ok / elapsed if elapsed > 0 else 0.0
        poll_hz = total_polls_ok / elapsed if elapsed > 0 else 0.0
        print("\n=== Summary ===")
        print(f"Elapsed: {elapsed:.3f} s")
        print(f"Successful reads (total polls): {total_polls_ok}")
        print(f"Cycles: {cycle_ok}")
        print(f"Errors: {err}")
        print(f"Cycle rate: {cycle_hz:.2f} cycles/s")
        print(f"Poll rate: {poll_hz:.2f} polls/s")
    finally:
        client.close()


if __name__ == "__main__":
    main()
