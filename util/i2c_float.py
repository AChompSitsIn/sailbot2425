#!/usr/bin/env python3
"""
Jetson I2C Float Sender
Sends floating point numbers to an Arduino via I2C.
"""

import smbus
import time
import struct
import random
import sys

# Configuration - CHANGE THESE VALUES based on your scanner results
BUS_NUM = 1  # The I2C bus number where your Arduino is connected
ARDUINO_ADDR = 0x09  # The I2C address of your Arduino (0x08 or 0x09)
DECIMAL_CMD = 2  # Command byte that tells Arduino we're sending a float

def send_float(bus, addr, cmd, value):
    """
    Sends a float value to the Arduino
    
    Args:
        bus: The SMBus object
        addr: The Arduino's I2C address
        cmd: The command byte to send
        value: The float value to send
    """
    # Convert float to bytes (4 bytes in IEEE 754 format)
    float_bytes = struct.pack('f', value)
    
    # Convert to list of integers (0-255)
    data = [b for b in float_bytes]
    
    try:
        # Send command followed by the float bytes
        bus.write_i2c_block_data(addr, cmd, data)
        time.sleep(1)
        print(f"Sent float: {value:.2f} to Arduino at 0x{addr:02X}")
        return True
    except Exception as e:
        print(f"Error sending data: {e}")
        return False

def main():
    print(f"Jetson I2C Float Sender (Bus: {BUS_NUM}, Arduino: 0x{ARDUINO_ADDR:02X})")
    print("====================================")
    print("Press Ctrl+C to exit")
    
    try:
        # Open the I2C bus
        bus = smbus.SMBus(BUS_NUM)
        time.sleep(1)        
        # Initial value
        value = 0.0
        
        # Main loop
        while True:
            # Generate a decimal number
            value += 0.1
            if value > 100:
                value = 0.0
                
            # Add some randomness
            rand_value = value + (random.random() * 0.1)
            
            # Send the value
            send_float(bus, ARDUINO_ADDR, DECIMAL_CMD, rand_value)
            
            # Wait before sending the next value
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            time.sleep(1)
            bus.close()
        except:
            pass
        print("I2C connection closed")

if __name__ == "__main__":
    main()
