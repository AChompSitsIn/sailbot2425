#!/usr/bin/env python3
"""
Simple Arduino Step Control Test Script
Sends variable step commands via I2C
"""

import smbus
import time
import sys

# I2C Configuration
ARDUINO_I2C_ADDR = 0x08
I2C_BUS = 1  # Use 1 for Raspberry Pi, may be 0 for older models

# Command codes
CMD_WINCH_CW_STEPS = 0x12
CMD_WINCH_CCW_STEPS = 0x13

class ArduinoStepControl:
    def __init__(self, bus_number=I2C_BUS, arduino_addr=ARDUINO_I2C_ADDR):
        try:
            self.bus = smbus.SMBus(bus_number)
            self.arduino_addr = arduino_addr
            print(f"✓ I2C connection established (Bus: {bus_number}, Address: 0x{arduino_addr:02X})")
        except Exception as e:
            print(f"✗ Failed to initialize I2C: {e}")
            sys.exit(1)
    
    def send_cw_steps(self, steps):
        """Send clockwise steps command (0-100,000 steps)"""
        if not (0 <= steps <= 100000):
            print(f"✗ Error: Steps must be between 0 and 100,000 (got {steps})")
            return False
        
        try:
            # Prepare command: 1 byte command + 4 bytes step count (big-endian)
            data = [CMD_WINCH_CW_STEPS]
            data.extend([(steps >> 24) & 0xFF, (steps >> 16) & 0xFF, 
                        (steps >> 8) & 0xFF, steps & 0xFF])
            
            self.bus.write_i2c_block_data(self.arduino_addr, data[0], data[1:])
            print(f"✓ Sent: CW {steps} steps")
            return True
        except Exception as e:
            print(f"✗ Error sending CW command: {e}")
            return False
    
    def send_ccw_steps(self, steps):
        """Send counter-clockwise steps command (0-100,000 steps)"""
        if not (0 <= steps <= 100000):
            print(f"✗ Error: Steps must be between 0 and 100,000 (got {steps})")
            return False
        
        try:
            # Prepare command: 1 byte command + 4 bytes step count (big-endian)
            data = [CMD_WINCH_CCW_STEPS]
            data.extend([(steps >> 24) & 0xFF, (steps >> 16) & 0xFF, 
                        (steps >> 8) & 0xFF, steps & 0xFF])
            
            self.bus.write_i2c_block_data(self.arduino_addr, data[0], data[1:])
            print(f"✓ Sent: CCW {steps} steps")
            return True
        except Exception as e:
            print(f"✗ Error sending CCW command: {e}")
            return False

def print_menu():
    """Display the test menu"""
    print("\n" + "=" * 40)
    print("    SIMPLE ARDUINO STEP CONTROL")
    print("=" * 40)
    print("1. Send CW steps")
    print("2. Send CCW steps")
    print("3. Quick test (10 steps each direction)")
    print("4. Exit")
    print("=" * 40)

def get_step_input():
    """Get step count from user with validation"""
    while True:
        try:
            steps = int(input("Enter number of steps (0-100,000): "))
            if 0 <= steps <= 100000:
                return steps
            else:
                print("Please enter a number between 0 and 100,000")
        except ValueError:
            print("Please enter a valid number")

def quick_test(controller):
    """Run a quick test with 10 steps in each direction"""
    print("\n🧪 Running quick test (10 steps each direction)...")
    
    print("Testing 10 CW steps...")
    controller.send_cw_steps(10)
    time.sleep(2)
    
    print("Testing 10 CCW steps...")
    controller.send_ccw_steps(10)
    time.sleep(2)
    
    print("✓ Quick test complete!")

def main():
    """Main program loop"""
    print("Starting Simple Arduino Step Control Test Script...")
    
    # Initialize connection
    controller = ArduinoStepControl()
    
    while True:
        print_menu()
        
        try:
            choice = input("\nEnter your choice (1-4): ").strip()
            
            if choice == '1':
                steps = get_step_input()
                controller.send_cw_steps(steps)
                
            elif choice == '2':
                steps = get_step_input()
                controller.send_ccw_steps(steps)
                
            elif choice == '3':
                quick_test(controller)
                
            elif choice == '4':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice. Please enter 1-4.")
                
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"✗ Unexpected error: {e}")

if __name__ == "__main__":
    main()
