#!/usr/bin/env python3
"""
Simple test script to read wind sensor data
"""

import serial
import time

# Use persistent symlink
SERIAL_PORT = '/dev/arduino_wind'
BAUD_RATE = 115200

try:
    # Open serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to wind sensor on {SERIAL_PORT}")
    print("Reading wind data... (Press Ctrl+C to stop)\n")
    
    while True:
        # Read a line from serial
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            
            # Check if it's wind data
            if line.startswith("WIND,"):
                try:
                    _, degrees = line.split(',')
                    wind_direction = float(degrees)
                    print(f"Wind Direction: {wind_direction}°")
                except ValueError:
                    print(f"Error parsing: {line}")
            
            # Check if it's an error
            elif line.startswith("ERROR,"):
                print(f"Sensor error: {line}")
            
            # Print any other data
            else:
                print(f"Other: {line}")
                
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
