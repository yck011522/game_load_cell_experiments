"""
Minimal Modbus RTU load-cell read speed test.

Default behavior follows the provided working frame pattern:
	Slaves: 1 and 2 (polled sequentially)
	Function: 0x03 (Read Holding Registers)
	Start register: 0x0001 ("second register" in 1-based docs)
	Quantity: 0x0003 registers

Response payload for quantity=3 is 6 bytes (3 registers).
This script decodes real-time weight as a signed 32-bit integer from the first
two returned registers (address 0001H and 0002H). Each read cycle does:
read sensor 1 -> read sensor 2 -> sum both into total_weight_1_2.
It tracks cycle rate (Hz) and can convert to grams using a linear formula:

	grams = (raw_signed - zero_count) * grams_per_count

Optionally, it auto-zeros at startup by averaging initial readings from each
sensor and subtracting those offsets from subsequent readings.

Dependency:
	pip install pyserial
"""

from __future__ import annotations

import argparse
import csv
import struct
import time
from datetime import datetime
from pathlib import Path

import serial


def modbus_crc16(data: bytes) -> int:
	"""Compute Modbus RTU CRC16 (little-endian in frame)."""
	crc = 0xFFFF
	for byte in data:
		crc ^= byte
		for _ in range(8):
			if crc & 0x0001:
				crc = (crc >> 1) ^ 0xA001
			else:
				crc >>= 1
	return crc & 0xFFFF


def build_read_request(slave: int, start_reg: int, qty_regs: int) -> bytes:
	pdu = struct.pack(">B B H H", slave, 0x03, start_reg, qty_regs)
	crc = modbus_crc16(pdu)
	return pdu + struct.pack("<H", crc)


def expected_response_len(qty_regs: int) -> int:
	# slave + function + byte_count + data(2*qty) + crc(2)
	return 1 + 1 + 1 + (2 * qty_regs) + 2


def read_exact(ser: serial.Serial, size: int) -> bytes:
	data = ser.read(size)
	if len(data) != size:
		raise TimeoutError(f"Expected {size} bytes, got {len(data)}")
	return data


def read_register_block(
	ser: serial.Serial, slave: int, start_reg: int = 0x0001, qty_regs: int = 3
) -> tuple[list[int], bytes, bytes]:
	request = build_read_request(slave, start_reg, qty_regs)
	ser.write(request)
	ser.flush()

	resp_len = expected_response_len(qty_regs)
	response = read_exact(ser, resp_len)

	body = response[:-2]
	recv_crc = struct.unpack("<H", response[-2:])[0]
	calc_crc = modbus_crc16(body)
	if recv_crc != calc_crc:
		raise ValueError(f"CRC mismatch: recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}")

	slave_rx, func, byte_count = response[0], response[1], response[2]
	if slave_rx != slave:
		raise ValueError(f"Unexpected slave id in response: {slave_rx}")
	if func & 0x80:
		exc_code = response[2]
		raise ValueError(f"Modbus exception from slave {slave}: code=0x{exc_code:02X}")
	if func != 0x03:
		raise ValueError(f"Unexpected function code: 0x{func:02X}")
	if byte_count != 2 * qty_regs:
		raise ValueError(f"Unexpected byte count: {byte_count}")

	data = response[3 : 3 + byte_count]
	regs = list(struct.unpack(">" + "H" * qty_regs, data))
	return regs, request, response


def read_register_block_with_retries(
	ser: serial.Serial,
	slave: int,
	start_reg: int,
	qty_regs: int,
	retries: int,
	retry_delay: float,
) -> tuple[list[int], bytes, bytes]:
	last_error: Exception | None = None
	for attempt in range(retries + 1):
		try:
			return read_register_block(ser, slave=slave, start_reg=start_reg, qty_regs=qty_regs)
		except TimeoutError as ex:
			last_error = ex
			# If a frame was late/partial, clear stale bytes before retry.
			ser.reset_input_buffer()
			if attempt < retries and retry_delay > 0:
				time.sleep(retry_delay)
		except Exception as ex:
			last_error = ex
			break
	if last_error is None:
		raise RuntimeError("Read failed for unknown reason")
	raise last_error


def read_single_u16(ser: serial.Serial, slave: int, reg_addr: int) -> int:
	regs, _req, _resp = read_register_block(ser, slave=slave, start_reg=reg_addr, qty_regs=1)
	return regs[0]


def u16_to_i16(value: int) -> int:
	return struct.unpack(">h", struct.pack(">H", value))[0]


def regs_to_i32(hi_u16: int, lo_u16: int) -> int:
	combined = (hi_u16 << 16) | lo_u16
	return struct.unpack(">i", struct.pack(">I", combined))[0]


def raw_to_grams(raw_signed: int, zero_count: float, grams_per_count: float) -> float:
	return (raw_signed - zero_count) * grams_per_count


def read_sensor_grams(
	ser: serial.Serial,
	slave: int,
	start_reg: int,
	qty_regs: int,
	retries: int,
	retry_delay: float,
	decimals: int,
	zero_count: float,
	grams_per_count: float,
) -> tuple[float, int]:
	regs, _req, _resp = read_register_block_with_retries(
		ser,
		slave=slave,
		start_reg=start_reg,
		qty_regs=qty_regs,
		retries=retries,
		retry_delay=retry_delay,
	)
	raw_i32 = regs_to_i32(regs[0], regs[1])
	scaled = raw_i32 / (10 ** decimals)
	grams = raw_to_grams(
		raw_signed=scaled,
		zero_count=zero_count,
		grams_per_count=grams_per_count,
	)
	return grams, raw_i32


def main() -> None:
	parser = argparse.ArgumentParser(description="Dual load-cell cycle-based Modbus RTU speed test")
	parser.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
	parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
	parser.add_argument(
		"--slaves",
		default="1,2",
		help="Comma-separated slave addresses to poll sequentially (default: 1,2)",
	)
	parser.add_argument("--seconds", type=float, default=10.0, help="Test duration in seconds")
	parser.add_argument("--timeout", type=float, default=0.05, help="Serial timeout in seconds")
	parser.add_argument(
		"--retries",
		type=int,
		default=1,
		help="Retries after timeout per request (default: 1)",
	)
	parser.add_argument(
		"--retry-delay",
		type=float,
		default=0.0005,
		help="Delay in seconds between retries (default: 0.0005)",
	)
	parser.add_argument(
		"--inter-request-delay",
		type=float,
		default=0.0,
		help="Delay in seconds after each request (default: 0)",
	)
	parser.add_argument(
		"--zero-count",
		type=float,
		default=0.0,
		help="Zero/tare raw count used in grams conversion",
	)
	parser.add_argument(
		"--grams-per-count",
		type=float,
		default=1.0,
		help="Grams per scaled count for linear conversion",
	)
	parser.add_argument(
		"--print-every",
		type=int,
		default=20,
		help="Print one sample every N successful cycles",
	)
	parser.add_argument(
		"--tare-samples",
		type=int,
		default=20,
		help="Number of startup cycles to average for auto-zero tare (default: 20)",
	)
	parser.add_argument(
		"--no-auto-zero",
		action="store_true",
		help="Disable startup auto-zero tare",
	)
	parser.add_argument(
		"--output-dir",
		default="logs",
		help="Directory for CSV and summary markdown output (default: logs)",
	)
	args = parser.parse_args()
	slaves = [int(x.strip()) for x in args.slaves.split(",") if x.strip()]
	if not slaves:
		raise ValueError("At least one slave address is required")

	run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
	output_dir = Path(args.output_dir)
	output_dir.mkdir(parents=True, exist_ok=True)
	csv_path = output_dir / f"experiment1_{run_id}.csv"
	summary_path = output_dir / f"experiment1_summary_{run_id}.md"

	start_reg = 0x0001
	qty_regs = 3

	for slave in slaves:
		req = build_read_request(slave, start_reg, qty_regs)
		print(f"Request frame slave {slave}:", req.hex(" ").upper())
	print("Expected sample (slave 1):", "01 03 00 01 00 03 54 0B")

	if len(slaves) != 2:
		raise ValueError("This cycle mode expects exactly two slave addresses, e.g. --slaves 1,2")

	s1 = slaves[0]
	s2 = slaves[1]

	total_polls_ok = 0
	cycle_ok = 0
	err = 0
	per_slave_ok = {s1: 0, s2: 0}
	per_slave_updates = {s1: 0, s2: 0}
	last_raw_by_slave: dict[int, int] = {}
	last_total_weight_1_2: float | None = None
	total_weight_updates = 0
	t0 = 0.0
	t_end = 0.0

	with serial.Serial(
		port=args.port,
		baudrate=args.baud,
		bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		timeout=args.timeout,
		write_timeout=args.timeout,
	) as ser, csv_path.open("w", newline="", encoding="utf-8") as csv_file:
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

		ser.reset_input_buffer()
		decimals_by_slave: dict[int, int] = {}
		for slave in slaves:
			try:
				dec_u16 = read_single_u16(ser, slave=slave, reg_addr=0x0000)
				decimals_by_slave[slave] = int(dec_u16)
			except Exception:
				# Fallback to documentation default if reading decimal config fails.
				decimals_by_slave[slave] = 2

		for slave in slaves:
			print(f"Slave {slave} decimal places: {decimals_by_slave[slave]}")
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
					g1, _r1 = read_sensor_grams(
						ser=ser,
						slave=s1,
						start_reg=start_reg,
						qty_regs=qty_regs,
						retries=args.retries,
						retry_delay=args.retry_delay,
						decimals=decimals_by_slave.get(s1, 2),
						zero_count=args.zero_count,
						grams_per_count=args.grams_per_count,
					)
					if args.inter_request_delay > 0:
						time.sleep(args.inter_request_delay)
					g2, _r2 = read_sensor_grams(
						ser=ser,
						slave=s2,
						start_reg=start_reg,
						qty_regs=qty_regs,
						retries=args.retries,
						retry_delay=args.retry_delay,
						decimals=decimals_by_slave.get(s2, 2),
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

		t0 = time.perf_counter()
		t_end = t0 + args.seconds

		while time.perf_counter() < t_end:
			if time.perf_counter() >= t_end:
				break

			try:
				grams1_raw, raw1_i32 = read_sensor_grams(
					ser=ser,
					slave=s1,
					start_reg=start_reg,
					qty_regs=qty_regs,
					retries=args.retries,
					retry_delay=args.retry_delay,
					decimals=decimals_by_slave.get(s1, 2),
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
					ser=ser,
					slave=s2,
					start_reg=start_reg,
					qty_regs=qty_regs,
					retries=args.retries,
					retry_delay=args.retry_delay,
					decimals=decimals_by_slave.get(s2, 2),
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
		"# Experiment 1 Summary",
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


if __name__ == "__main__":
	main()
