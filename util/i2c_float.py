#!/usr/bin/env python3

import sys
import struct
from smbus2 import SMBus

# === CONFIGURATION ===
I2C_BUS = 1               # Bus 1 = /dev/i2c-1
ARDUINO_ADDR = 0x08       # Must match Arduino define
DECIMAL_CMD = 0x02        # Command byte must match DECIMAL_CMD in Arduino

def send_float(slave_addr, float_value):
    # Pack the float into 4 bytes (little endian to match Arduino)
    float_bytes = struct.pack('<f', float_value)

    # Prepend command byte
    message = [DECIMAL_CMD] + list(float_bytes)

    # Send via I2C
    with SMBus(I2C_BUS) as bus:
        bus.write_i2c_block_data(slave_addr, message[0], message[1:])
        print(f"Sent float {float_value} to I2C address 0x{slave_addr:02X}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: sudo python3 i2c_float.py <hex address> <float value>")
        sys.exit(1)

    addr = int(sys.argv[1], 16)
    value = float(sys.argv[2])
    send_float(addr, value)
