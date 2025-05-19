#!/usr/bin/env python3
import smbus
import time

def scan_i2c_bus(bus_num):
    print(f"Scanning I2C bus {bus_num} with read test...")
    try:
        bus = smbus.SMBus(bus_num)
        
        found_devices = []
        phantom_devices = []
        
        # Test addresses 0x08 and 0x09 first (your target devices)
        for addr in [0x08, 0x09]:
            print(f"Testing address 0x{addr:02X}...")
            try:
                bus.read_byte(addr)
                print(f"  - Response received!")
                found_devices.append(addr)
            except OSError as e:
                if e.errno == 121:  # Remote I/O error
                    print(f"  - Detected but not responding (phantom)")
                    phantom_devices.append(addr)
                else:
                    print(f"  - No device present")
        
        bus.close()
        
        if found_devices:
            print(f"Found working devices: {[f'0x{x:02X}' for x in found_devices]}")
        if phantom_devices:
            print(f"Found non-responsive devices: {[f'0x{x:02X}' for x in phantom_devices]}")
            
    except Exception as e:
        print(f"Error scanning bus {bus_num}: {e}")

# Scan all three buses
for bus in [1, 11, 12]:
    print(f"\n{'='*40}")
    scan_i2c_bus(bus)
