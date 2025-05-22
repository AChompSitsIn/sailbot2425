#!/usr/bin/env python3
import sys
from smbus2 import SMBus

I2C_BUS = 1
ARDUINO_ADDR = 0x09
SERVO_CMD = 0x20

def send_servo_angle(angle):
    angle = max(0, min(180, int(angle)))  # Clamp to safe range
    with SMBus(I2C_BUS) as bus:
        bus.write_i2c_block_data(ARDUINO_ADDR, SERVO_CMD, [angle])
        print(f"Sent servo angle {angle}° to Arduino at 0x{ARDUINO_ADDR:02X}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: sudo python3 i2c_rudder_send.py <angle>")
        sys.exit(1)

    send_servo_angle(sys.argv[1])
