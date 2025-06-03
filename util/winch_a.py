#!/usr/bin/env python3

import smbus
import math

# I2C Configuration
WINCH_ADDRESS = 0x08
I2C_BUS = 1

# Physical parameters
BOOM_LENGTH = 28
WINCH_TO_MAST = 40
SPOOL_RADIUS = 3
GEAR_RATIO = 10
STEPS_PER_REVOLUTION = 1600

bus = smbus.SMBus(I2C_BUS)

def angle_to_steps(angle):
    abs_angle = abs(angle)
    if abs_angle == 0:
        return 0

    length = math.sqrt(BOOM_LENGTH**2 + WINCH_TO_MAST**2 - 2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(abs_angle)))
    steps = int(length / (2 * math.pi * SPOOL_RADIUS / GEAR_RATIO / STEPS_PER_REVOLUTION * 2))

    return steps

def send_steps(steps):
    high_byte = (steps >> 8) & 0xFF
    low_byte = steps & 0xFF
    bus.write_i2c_block_data(WINCH_ADDRESS, 0, [high_byte, low_byte])

while True:
    try:
        angle = float(input("Enter angle: "))
        steps = angle_to_steps(angle)
        print(f"Sending {steps} steps")
        send_steps(steps)
    except KeyboardInterrupt:
        break
    except ValueError:
        print("Invalid angle")
