#!/usr/bin/env python3

import struct
from smbus2 import SMBus

I2C_BUS = 1
ARDUINO_ADDR = 0x09

def read_wind_float():
    with SMBus(I2C_BUS) as bus:
        data = bus.read_i2c_block_data(ARDUINO_ADDR, 0x00, 4)
        value = struct.unpack('<f', bytes(data))[0]
        print(f"Wind Direction: {value:.2f}°")

if __name__ == "__main__":
    read_wind_float()
