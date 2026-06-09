"""
Detect a Modbus RTU load cell, change its slave address, and verify communication.

Default workflow:
1) Connect on COM12 and probe slave address 1 by reading holding register 0x0000.
2) Prompt the user for a new slave ID (1..32).
3) Write the new slave ID to holding register 0x0040 (decimal 64).
4) Verify the device responds on the new slave ID.

Notes:
- Address register mapping provided by user/spec:
  0040H (64) = communication address, R/W, uint16, range 1..32.
- Uses Modbus function 03 for reads and function 06 (single register write).

Dependency:
    pip install pymodbus pyserial
"""

from __future__ import annotations

import argparse
import inspect
import sys
import time

from pymodbus.client import ModbusSerialClient


ADDRESS_REGISTER = 0x0040
PROBE_REGISTER = 0x0000
MIN_SLAVE_ID = 1
MAX_SLAVE_ID = 32


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

    return fn(address, count, slave)


def write_single_register_compat(
    client: ModbusSerialClient, address: int, value: int, slave: int
):
    """Call write_register across pymodbus API variants."""
    fn = client.write_register
    params = inspect.signature(fn).parameters

    if "slave" in params:
        return fn(address=address, value=value, slave=slave)
    if "unit" in params:
        return fn(address=address, value=value, unit=slave)
    if "device_id" in params:
        return fn(address=address, value=value, device_id=slave)

    return fn(address, value, slave)


def probe_device(client: ModbusSerialClient, slave: int) -> tuple[bool, str]:
    """Try a simple register read to confirm the device is responding."""
    try:
        rr = read_holding_registers_compat(client, address=PROBE_REGISTER, count=1, slave=slave)
        if rr.isError():
            return False, f"Modbus error: {rr}"
        value = rr.registers[0]
        return True, f"Read OK (reg 0x{PROBE_REGISTER:04X} = {value})"
    except Exception as ex:
        return False, str(ex)


def prompt_new_slave_id(old_slave: int) -> int:
    while True:
        raw = input(f"Enter new slave ID ({MIN_SLAVE_ID}-{MAX_SLAVE_ID}, current {old_slave}): ").strip()
        if not raw:
            print("Please enter a value.")
            continue

        try:
            new_id = int(raw)
        except ValueError:
            print("Invalid input. Please enter a number.")
            continue

        if not (MIN_SLAVE_ID <= new_id <= MAX_SLAVE_ID):
            print(f"Out of range. Must be {MIN_SLAVE_ID}..{MAX_SLAVE_ID}.")
            continue

        return new_id


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Detect load cell on Modbus RTU, change slave ID, and verify response"
    )
    parser.add_argument("--port", default="COM12", help="Serial port (default: COM12)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial timeout in seconds")
    parser.add_argument("--old-id", type=int, default=1, help="Current slave ID (default: 1)")
    parser.add_argument(
        "--settle-delay",
        type=float,
        default=0.2,
        help="Delay in seconds after writing new ID before re-probe",
    )
    args = parser.parse_args()

    if not (MIN_SLAVE_ID <= args.old_id <= MAX_SLAVE_ID):
        raise ValueError(f"--old-id must be {MIN_SLAVE_ID}..{MAX_SLAVE_ID}")

    client = ModbusSerialClient(
        port=args.port,
        baudrate=args.baud,
        timeout=args.timeout,
        stopbits=1,
        bytesize=8,
        parity="N",
    )

    print(f"Opening {args.port} @ {args.baud}...")
    if not client.connect():
        print(f"Failed to open serial port {args.port}")
        sys.exit(1)

    try:
        print(f"Probing for load cell at slave ID {args.old_id}...")
        ok, msg = probe_device(client, args.old_id)
        if not ok:
            print(f"No responding device at slave {args.old_id}: {msg}")
            sys.exit(2)

        print(f"Device detected at slave {args.old_id}. {msg}")

        new_id = prompt_new_slave_id(args.old_id)
        if new_id == args.old_id:
            print("New ID matches current ID; nothing to change.")
            verify_ok, verify_msg = probe_device(client, args.old_id)
            if verify_ok:
                print(f"Verification OK at slave {args.old_id}: {verify_msg}")
                sys.exit(0)
            print(f"Verification failed at slave {args.old_id}: {verify_msg}")
            sys.exit(3)

        print(
            f"Writing new slave ID {new_id} to register 0x{ADDRESS_REGISTER:04X} "
            f"on current slave {args.old_id}..."
        )
        wr = write_single_register_compat(
            client,
            address=ADDRESS_REGISTER,
            value=new_id,
            slave=args.old_id,
        )
        if wr.isError():
            print(f"Write failed: {wr}")
            sys.exit(4)

        time.sleep(max(args.settle_delay, 0.0))

        print(f"Verifying response on new slave ID {new_id}...")
        verify_ok, verify_msg = probe_device(client, new_id)
        if not verify_ok:
            print(f"No response at new slave {new_id}: {verify_msg}")
            sys.exit(5)

        print(f"Success: device now responds at slave {new_id}. {verify_msg}")

        old_ok, old_msg = probe_device(client, args.old_id)
        if old_ok:
            print(
                f"Warning: old slave {args.old_id} still responds ({old_msg}). "
                "If only one device is connected, power-cycle and retry verification."
            )
        else:
            print(f"Old slave {args.old_id} no longer responds, as expected.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
