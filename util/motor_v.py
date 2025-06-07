#!/usr/bin/env python3
"""
Simple test script to send rudder and winch commands
"""

import serial
import struct
import time

# Use persistent symlink
SERIAL_PORT = '/dev/arduino_control'
BAUD_RATE = 115200

# Command codes
SERVO_CMD = 0x20
CMD_WINCH_CW_STEPS = 0x12
CMD_WINCH_CCW_STEPS = 0x13
CMD_ENABLE_MOTOR = 0x14
CMD_DISABLE_MOTOR = 0x15

def send_rudder_angle(ser, angle):
    """Send rudder angle command (0-180)"""
    if 0 <= angle <= 180:
        command = bytes([SERVO_CMD, angle])
        ser.write(command)
        print(f"Sent rudder angle: {angle}°")
    else:
        print(f"Error: Angle must be 0-180, got {angle}")

def send_winch_cw(ser, steps):
    """Send winch clockwise command"""
    if 0 <= steps <= 100000:
        command = bytes([CMD_WINCH_CW_STEPS]) + struct.pack('>I', steps)
        ser.write(command)
        print(f"Sent winch CW: {steps} steps")
    else:
        print(f"Error: Steps must be 0-100000, got {steps}")

def send_winch_ccw(ser, steps):
    """Send winch counter-clockwise command"""
    if 0 <= steps <= 100000:
        command = bytes([CMD_WINCH_CCW_STEPS]) + struct.pack('>I', steps)
        ser.write(command)
        print(f"Sent winch CCW: {steps} steps")
    else:
        print(f"Error: Steps must be 0-100000, got {steps}")

def enable_motor(ser):
    """Enable winch motor"""
    ser.write(bytes([CMD_ENABLE_MOTOR]))
    print("Sent motor enable")

def disable_motor(ser):
    """Disable winch motor"""
    ser.write(bytes([CMD_DISABLE_MOTOR]))
    print("Sent motor disable")

def read_response(ser, timeout=1):
    """Read any response from Arduino"""
    ser.timeout = timeout
    if ser.in_waiting > 0:
        response = ser.readline().decode('utf-8').strip()
        print(f"Arduino says: {response}")

# Main test
try:
    # Open serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to control system on {SERIAL_PORT}")
    print("Running control tests...\n")
    
    # Test 1: Rudder control
    print("=== Testing Rudder ===")
    for angle in [90, 45, 135, 90, 57]:
        send_rudder_angle(ser, angle)
        time.sleep(0.5)
        read_response(ser)
        time.sleep(0.5)
    
    # Test 2: Winch control
    print("\n=== Testing Winch ===")
    
    # Enable motor
    enable_motor(ser)
    time.sleep(0.5)
    read_response(ser)
    
    # Move CW
    send_winch_cw(ser, 1000)
    time.sleep(0.5)
    read_response(ser)
    time.sleep(2)  # Wait for movement
    
    # Move CCW
    send_winch_ccw(ser, 500)
    time.sleep(0.5)
    read_response(ser)
    time.sleep(2)  # Wait for movement
    
    # Interactive mode
    print("\n=== Interactive Mode ===")
    print("Commands:")
    print("  r <angle>    - Set rudder angle (0-180)")
    print("  cw <steps>   - Winch clockwise")
    print("  ccw <steps>  - Winch counter-clockwise")
    print("  enable       - Enable motor")
    print("  disable      - Disable motor")
    print("  quit         - Exit")
    
    while True:
        cmd = input("\nEnter command: ").strip().lower()
        
        if cmd.startswith('r '):
            try:
                angle = int(cmd.split()[1])
                send_rudder_angle(ser, angle)
                time.sleep(0.1)
                read_response(ser)
            except (IndexError, ValueError):
                print("Usage: r <angle>")
                
        elif cmd.startswith('cw '):
            try:
                steps = int(cmd.split()[1])
                send_winch_cw(ser, steps)
                time.sleep(0.1)
                read_response(ser)
            except (IndexError, ValueError):
                print("Usage: cw <steps>")
                
        elif cmd.startswith('ccw '):
            try:
                steps = int(cmd.split()[1])
                send_winch_ccw(ser, steps)
                time.sleep(0.1)
                read_response(ser)
            except (IndexError, ValueError):
                print("Usage: ccw <steps>")
                
        elif cmd == 'enable':
            enable_motor(ser)
            time.sleep(0.1)
            read_response(ser)
            
        elif cmd == 'disable':
            disable_motor(ser)
            time.sleep(0.1)
            read_response(ser)
            
        elif cmd == 'quit':
            break
            
        else:
            print("Unknown command")
            
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
