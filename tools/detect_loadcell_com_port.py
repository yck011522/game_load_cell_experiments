"""
Auto-detect the first COM port with a responding load cell at Modbus slave address 1.

Behavior:
1) Enumerate local serial COM ports.
2) Open each port with Modbus RTU settings.
3) Probe slave address 1 by reading holding register 0x0000.
4) Print the first matching COM port and exit.

Dependency:
    pip install pymodbus pyserial
"""

from __future__ import annotations

import argparse
import inspect
import re
import sys

from pymodbus.client import ModbusSerialClient
from serial.tools import list_ports


PROBE_SLAVE = 1
PROBE_REGISTER = 0x0000
PROBE_COUNT = 1


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


def com_sort_key(device_name: str) -> tuple[int, str]:
    """Sort COM ports numerically first (COM3 before COM12)."""
    match = re.match(r"^COM(\d+)$", device_name.upper())
    if match:
        return int(match.group(1)), device_name
    return 10**9, device_name


def probe_port(port: str, baud: int, timeout: float, slave: int) -> bool:
    client = ModbusSerialClient(
        port=port,
        baudrate=baud,
        timeout=timeout,
        stopbits=1,
        bytesize=8,
        parity="N",
    )

    if not client.connect():
        return False

    try:
        rr = read_holding_registers_compat(
            client,
            address=PROBE_REGISTER,
            count=PROBE_COUNT,
            slave=slave,
        )
        return not rr.isError()
    except Exception:
        return False
    finally:
        client.close()


def detect_first_port(baud: int, timeout: float, slave: int) -> str | None:
    ports = [p.device for p in list_ports.comports()]
    ports = sorted(ports, key=com_sort_key)

    for port in ports:
        if probe_port(port=port, baud=baud, timeout=timeout, slave=slave):
            return port
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Detect first COM port with load cell response at slave address 1"
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--timeout", type=float, default=0.08, help="Serial timeout in seconds")
    parser.add_argument("--slave", type=int, default=PROBE_SLAVE, help="Slave ID to probe")
    args = parser.parse_args()

    found = detect_first_port(baud=args.baud, timeout=args.timeout, slave=args.slave)
    if found is None:
        print("")
        sys.exit(1)

    # Print only the first found COM port for easy scripting.
    print(found)


if __name__ == "__main__":
    main()
