#!/usr/bin/env python3

import sys
from smbus2 import SMBus

I2C_BUS = 1
ARDUINO_ADDR = 0x08

# Command definitions
CMD_WINCH_CW = 0x10
CMD_WINCH_CCW = 0x11

def send_command(cmd_byte):
    with SMBus(I2C_BUS) as bus:
        bus.write_byte(ARDUINO_ADDR, cmd_byte)
        print(f"Sent command 0x{cmd_byte:02X} to Arduino at 0x{ARDUINO_ADDR:02X}")

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ["cw", "ccw"]:
        print("Usage: sudo python3 i2c_winch_send.py <cw|ccw>")
        sys.exit(1)

    command = CMD_WINCH_CW if sys.argv[1] == "cw" else CMD_WINCH_CCW
    send_command(command)
