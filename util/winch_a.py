#!/usr/bin/env python3
"""
Quick script to set sail angle
Usage: python3 quick_sail_test.py <angle>
"""

import serial
import struct
import math
import time
import sys

# Serial configuration
SERIAL_PORT = '/dev/arduino_control'
BAUD_RATE = 115200

# Command codes
CMD_WINCH_CW_STEPS = 0x12   # Let sail out
CMD_WINCH_CCW_STEPS = 0x13  # Bring sail in

# Sail parameters
BOOM_LENGTH = 28
WINCH_TO_MAST = 40
SPOOL_RADIUS = 3
GEAR_RATIO = 10
STEPS_PER_REVOLUTION = 1600

def angle_to_steps(angle):
    """Convert angle to steps"""
    angle = abs(angle)
    angle = max(0.0, min(88.0, angle))
    
    length = math.sqrt(
        BOOM_LENGTH**2 + WINCH_TO_MAST**2 - 
        2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(angle))
    )
    
    return int(length / (2 * math.pi * SPOOL_RADIUS * GEAR_RATIO * STEPS_PER_REVOLUTION * 2))

def set_sail(angle_degrees):
    """Set sail to specified angle"""
    try:
        # Connect
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Arduino reset
        
        # Calculate steps
        steps = angle_to_steps(angle_degrees)
        print(f"Setting sail to {angle_degrees}° ({steps} steps)")
        
        # For this simple test, assume we're starting from 0
        # So we always move CW (let out) to reach the angle
        command = bytes([CMD_WINCH_CW_STEPS]) + struct.pack('>I', steps)
        ser.write(command)
        
        # Read response
        time.sleep(0.5)
        while ser.in_waiting > 0:
            print(f"Arduino: {ser.readline().decode('utf-8').strip()}")
        
        ser.close()
        print("Done!")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 quick_sail_test.py <angle>")
        print("Example: python3 quick_sail_test.py 45")
        sys.exit(1)
    
    try:
        angle = float(sys.argv[1])
        if not 0 <= angle <= 88:
            print("Angle must be between 0 and 88 degrees")
            sys.exit(1)
        
        set_sail(angle)
        
    except ValueError:
        print("Please provide a valid number")
        sys.exit(1)
