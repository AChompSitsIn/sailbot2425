#!/usr/bin/env python3
"""
Improved I2C Scanner for Jetson Orin
This script scans all available I2C buses with detailed debugging information
to help diagnose connection issues with Arduino devices.
"""

import smbus
import time
import os
import subprocess
import sys

def check_i2c_tools():
    """Check if i2c-tools package is installed and provide information about i2c interface"""
    print("Checking I2C interface configuration...")
    
    # Check if i2c-tools is installed
    try:
        result = subprocess.run(['which', 'i2cdetect'], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
        if result.returncode != 0:
            print("i2cdetect not found. Please install i2c-tools:")
            print("  sudo apt-get install -y i2c-tools")
            return False
    except Exception as e:
        print(f"Error checking for i2cdetect: {e}")
        return False
    
    # Check if user has permission to access i2c devices
    try:
        result = subprocess.run(['ls', '-la', '/dev/i2c*'], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
        print("I2C devices and permissions:")
        print(result.stdout)
        
        # Check if current user is in i2c group
        user = os.getenv('USER')
        groups_result = subprocess.run(['groups', user], 
                                      stdout=subprocess.PIPE, 
                                      stderr=subprocess.PIPE, 
                                      text=True)
        groups = groups_result.stdout.split()
        if 'i2c' not in groups:
            print(f"WARNING: User '{user}' is not in the i2c group.")
            print("To add user to i2c group, run:")
            print(f"  sudo usermod -a -G i2c {user}")
            print("  sudo reboot")
    except Exception as e:
        print(f"Error checking i2c permissions: {e}")
    
    return True

def get_available_buses():
    """Get list of available I2C buses"""
    available_buses = []
    try:
        for i in range(20):  # Check buses 0-19
            if os.path.exists(f"/dev/i2c-{i}"):
                available_buses.append(i)
    except Exception as e:
        print(f"Error checking available buses: {e}")
    
    return available_buses

def scan_i2c_bus(bus_num, scan_all=False):
    """Scan an I2C bus for devices and return a list of addresses found"""
    found_devices = []
    
    try:
        print(f"Opening I2C bus {bus_num}...")
        bus = smbus.SMBus(bus_num)
        
        # Address range to scan
        start_addr = 0x03
        end_addr = 0x77
        
        # For quick scan, prioritize common Arduino addresses
        if not scan_all:
            addresses = [0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
            print(f"Quick-scanning common Arduino addresses on bus {bus_num}...")
        else:
            addresses = range(start_addr, end_addr + 1)
            print(f"Full-scanning all addresses on bus {bus_num} (0x{start_addr:02X}-0x{end_addr:02X})...")
        
        for addr in addresses:
            try:
                # Try to read a byte from the device
                print(f"  Probing address 0x{addr:02X}...", end='', flush=True)
                bus.read_byte(addr)
                print(" FOUND!")
                found_devices.append(addr)
            except OSError as e:
                print(f" No device (error: {e})")
            except Exception as e:
                print(f" Error: {e}")
        
        bus.close()
    except FileNotFoundError:
        print(f"  Bus {bus_num} doesn't exist or can't be opened")
    except PermissionError:
        print(f"  Permission denied accessing bus {bus_num}. Try running with sudo.")
    except Exception as e:
        print(f"  Error accessing bus {bus_num}: {e}")
    
    return found_devices

def compare_with_i2cdetect(bus_num, our_devices):
    """Compare our results with i2cdetect for verification"""
    try:
        print(f"\nRunning system i2cdetect on bus {bus_num} for comparison:")
        result = subprocess.run(['i2cdetect', '-y', str(bus_num)], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
        if result.returncode == 0:
            print(result.stdout)
            
            # Parse i2cdetect output to find devices
            i2cdetect_devices = []
            for line in result.stdout.splitlines()[1:]:  # Skip header line
                parts = line.split(':')
                if len(parts) > 1:
                    row_values = parts[1].strip().split()
                    row_base = int(parts[0], 16) * 16  # Convert row header to int
                    for i, val in enumerate(row_values):
                        if val != "--":
                            addr = row_base + i
                            i2cdetect_devices.append(addr)
            
            # Compare results
            if set(our_devices) == set(i2cdetect_devices):
                print("✓ Our scan matches i2cdetect results")
            else:
                print("! Discrepancy between our scan and i2cdetect:")
                if set(our_devices) - set(i2cdetect_devices):
                    print("  Addresses we found but i2cdetect didn't:", 
                          [f"0x{a:02X}" for a in (set(our_devices) - set(i2cdetect_devices))])
                if set(i2cdetect_devices) - set(our_devices):
                    print("  Addresses i2cdetect found but we didn't:", 
                          [f"0x{a:02X}" for a in (set(i2cdetect_devices) - set(our_devices))])
        else:
            print(f"Error running i2cdetect: {result.stderr}")
    except Exception as e:
        print(f"Error comparing with i2cdetect: {e}")

def try_communicating_with_arduino(bus_num, addr):
    """Try to communicate with an Arduino at the specified address"""
    print(f"\nAttempting to communicate with Arduino at 0x{addr:02X} on bus {bus_num}...")
    
    try:
        bus = smbus.SMBus(bus_num)
        
        # First try just reading a byte
        try:
            print("  Reading a byte...", end='', flush=True)
            value = bus.read_byte(addr)
            print(f" SUCCESS! Received: {value}")
        except Exception as e:
            print(f" FAILED: {e}")
        
        # Try writing a command
        try:
            print("  Writing command (value 2)...", end='', flush=True)
            bus.write_byte(addr, 2)
            print(" SUCCESS!")
        except Exception as e:
            print(f" FAILED: {e}")
        
        # Try writing a float (value 90.0 to set servo to center position)
        try:
            import struct
            print("  Writing float value (90.0)...", end='', flush=True)
            float_bytes = list(struct.pack('f', 90.0))
            bus.write_i2c_block_data(addr, 2, float_bytes)
            print(" SUCCESS!")
        except Exception as e:
            print(f" FAILED: {e}")
        
        bus.close()
        
    except Exception as e:
        print(f"  Error communicating with device: {e}")

def main():
    """Main function to scan all I2C buses"""
    print("Improved I2C Bus Scanner for Jetson Orin")
    print("=======================================")
    
    # Check i2c-tools and permissions
    if not check_i2c_tools():
        print("Please install i2c-tools and configure permissions before continuing.")
    
    # Get available buses
    available_buses = get_available_buses()
    
    if not available_buses:
        print("\nNo I2C buses found on this system.")
        return
    
    print(f"\nFound {len(available_buses)} I2C buses: {available_buses}")
    
    # Ask for quick scan or full scan
    scan_mode = input("\nPerform quick scan (q) or full scan (f)? [q/f]: ").lower()
    full_scan = scan_mode == 'f'
    
    # Store all results
    results = {}
    
    # Scan each bus
    for bus_num in available_buses:
        print(f"\n{'-' * 40}")
        print(f"Scanning bus {bus_num}...")
        devices = scan_i2c_bus(bus_num, full_scan)
        
        if devices:
            results[bus_num] = devices
            print(f"  Found {len(devices)} devices on bus {bus_num}: {', '.join([f'0x{d:02X}' for d in devices])}")
            
            # Compare with i2cdetect
            compare_with_i2cdetect(bus_num, devices)
            
            # Try to communicate with Arduino addresses
            for addr in devices:
                if addr in [0x08, 0x09]:  # Common Arduino addresses
                    try_communicating_with_arduino(bus_num, addr)
        else:
            print(f"  No devices found on bus {bus_num}")
            # Run i2cdetect anyway for verification
            compare_with_i2cdetect(bus_num, [])
    
    # Summary
    print("\n" + "=" * 50)
    print("SCAN RESULTS SUMMARY")
    print("=" * 50)
    
    if results:
        print("Found devices on the following buses:")
        for bus_num, devices in results.items():
            print(f"Bus {bus_num}: {len(devices)} devices at addresses {', '.join([f'0x{d:02X}' for d in devices])}")
            
        print("\nTo use these buses in your code, use:")
        for bus_num, devices in results.items():
            for addr in devices:
                print(f"BUS_NUM = {bus_num}, DEVICE_ADDR = 0x{addr:02X}  # For device at address 0x{addr:02X}")
    else:
        print("No I2C devices found on any bus.")
        print("Possible issues to check:")
        print("1. Verify the wiring between Jetson and Arduino")
        print("2. Make sure Arduino has the correct I2C slave address")
        print("3. Check that Arduino is powered and running the I2C slave code")
        print("4. Ensure pull-up resistors are present on SDA and SCL lines")
        print("5. Try using shorter wires for the I2C connection")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScan interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")