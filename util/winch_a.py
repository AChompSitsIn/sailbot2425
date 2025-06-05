#!/usr/bin/env python3
"""
Simple Arduino Step Control Test Script
Sends variable step commands via I2C with sail angle conversion
"""

import smbus
import time
import sys
import math

# I2C Configuration
ARDUINO_I2C_ADDR = 0x08
I2C_BUS = 1  # Use 1 for Raspberry Pi, may be 0 for older models

# Command codes
CMD_WINCH_CW_STEPS = 0x12   # Clockwise - let sail out
CMD_WINCH_CCW_STEPS = 0x13  # Counter-clockwise - bring sail in

# Sail angle conversion parameters (corrected values)
BOOM_LENGTH = 48          # inches
WINCH_TO_MAST = 44        # inches
SPOOL_RADIUS = 1.5        # inches
GEAR_RATIO = 5
STEPS_PER_REVOLUTION = 1600

class ArduinoStepControl:
    def __init__(self, bus_number=I2C_BUS, arduino_addr=ARDUINO_I2C_ADDR):
        try:
            self.bus = smbus.SMBus(bus_number)
            self.arduino_addr = arduino_addr
            # Track current sail position
            self.current_angle = 0.0  # Start at 0 degrees
            self.current_steps = 0    # Start at 0 steps
            print(f"I2C connection established (Bus: {bus_number}, Address: 0x{arduino_addr:02X})")
            print(f"Initial sail position: {self.current_angle}° (0 steps)")
        except Exception as e:
            print(f"Failed to initialize I2C: {e}")
            sys.exit(1)
    
    def move_to_angle(self, target_angle):
        """
        Move sail from current position to target angle.
        Calculates the difference and sends appropriate steps.
        """
        # Take absolute value since positive/negative angles produce same result
        target_angle = abs(target_angle)
        
        # Clamp angle to reasonable range
        target_angle = max(0.0, min(88.0, target_angle))
        
        # Calculate target steps
        target_steps = self.angle_to_steps(target_angle)
        
        # Calculate difference in steps
        steps_difference = target_steps - self.current_steps
        
        print(f"\nMoving sail from {self.current_angle}° to {target_angle}°")
        print(f"Current steps: {self.current_steps}, Target steps: {target_steps}")
        print(f"Steps difference: {steps_difference}")
        
        if steps_difference == 0:
            print("Sail already at target position!")
            return True
        
        # Determine direction and send command
        if steps_difference > 0:
            # Need to let sail out (CW)
            print(f"Sending CW {abs(steps_difference)} steps (let out)")
            success = self.send_cw_steps(abs(steps_difference))
        else:
            # Need to bring sail in (CCW) 
            print(f"Sending CCW {abs(steps_difference)} steps (bring in)")
            success = self.send_ccw_steps(abs(steps_difference))
        
        # Update tracked position if successful
        if success:
            self.current_angle = target_angle
            self.current_steps = target_steps
            print(f"Updated sail position: {self.current_angle}° ({self.current_steps} steps)")
        
        return success
    
    def set_current_position(self, angle):
        """Set the current sail position (for calibration)"""
        angle = abs(angle)
        angle = max(0.0, min(88.0, angle))
        
        self.current_angle = angle
        self.current_steps = self.angle_to_steps(angle)
        print(f"Set current sail position: {self.current_angle}° ({self.current_steps} steps)")
    
    def get_current_position(self):
        """Show current tracked sail position"""
        print(f"Current sail position: {self.current_angle}° ({self.current_steps} steps)")
        return self.current_angle, self.current_steps
    
    def angle_to_steps(self, target_angle_degrees):
        """
        Convert target sail angle in degrees to motor steps using winch math.
        
        Args:
            target_angle_degrees: The desired sail angle in degrees
            
        Returns:
            The corresponding number of motor steps
        """
        # Take absolute value since positive/negative angles produce same result
        target_angle_degrees = abs(target_angle_degrees)
        
        # Clamp angle to reasonable range
        target_angle_degrees = max(0.0, min(88.0, target_angle_degrees))
        
        # Calculate length using law of cosines
        length = math.sqrt(
            BOOM_LENGTH**2 + WINCH_TO_MAST**2 - 
            2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(target_angle_degrees))
        )
        
        # Convert to steps: length -> spool rotations -> motor rotations -> steps
        spool_circumference = 2 * math.pi * SPOOL_RADIUS
        spool_rotations = length / spool_circumference
        motor_rotations = spool_rotations * GEAR_RATIO
        target_steps = int(motor_rotations * STEPS_PER_REVOLUTION)
        
        return target_steps
    
    def send_cw_steps(self, steps, update_tracking=False):
        """Send clockwise steps command (let sail out)"""
        if not (0 <= steps <= 100000):
            print(f"Error: Steps must be between 0 and 100,000 (got {steps})")
            return False
        
        try:
            # Prepare command: 1 byte command + 4 bytes step count (big-endian)
            data = [CMD_WINCH_CW_STEPS]
            data.extend([(steps >> 24) & 0xFF, (steps >> 16) & 0xFF, 
                        (steps >> 8) & 0xFF, steps & 0xFF])
            
            self.bus.write_i2c_block_data(self.arduino_addr, data[0], data[1:])
            print(f"Sent: CW {steps} steps (let sail out)")
            
            # Update tracking if requested
            if update_tracking:
                self.current_steps += steps
                # Convert back to angle (rough estimate)
                print(f"WARNING: Position tracking may be inaccurate after raw step commands")
            
            return True
        except Exception as e:
            print(f"Error sending CW command: {e}")
            return False
    
    def send_ccw_steps(self, steps, update_tracking=False):
        """Send counter-clockwise steps command (bring sail in)"""
        if not (0 <= steps <= 100000):
            print(f"Error: Steps must be between 0 and 100,000 (got {steps})")
            return False
        
        try:
            # Prepare command: 1 byte command + 4 bytes step count (big-endian)
            data = [CMD_WINCH_CCW_STEPS]
            data.extend([(steps >> 24) & 0xFF, (steps >> 16) & 0xFF, 
                        (steps >> 8) & 0xFF, steps & 0xFF])
            
            self.bus.write_i2c_block_data(self.arduino_addr, data[0], data[1:])
            print(f"Sent: CCW {steps} steps (bring sail in)")
            
            # Update tracking if requested
            if update_tracking:
                self.current_steps -= steps
                if self.current_steps < 0:
                    self.current_steps = 0
                print(f"WARNING: Position tracking may be inaccurate after raw step commands")
            
            return True
        except Exception as e:
            print(f"Error sending CCW command: {e}")
            return False

def print_menu():
    """Display the test menu"""
    print("\n" + "=" * 50)
    print("    SAIL WINCH CONTROL TEST SCRIPT")
    print("=" * 50)
    print("1. Move to angle (tracked positioning)")
    print("2. Show current position")
    print("3. Set current position (calibration)")
    print("4. Send raw CW steps")
    print("5. Send raw CCW steps") 
    print("6. Calculate steps from angle (preview)")
    print("7. Quick test (10 steps each direction)")
    print("8. Exit")
    print("=" * 50)

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

def get_angle_input():
    """Get angle from user with validation"""
    while True:
        try:
            angle = float(input("Enter sail angle in degrees (0-88): "))
            if 0 <= abs(angle) <= 88:
                return angle
            else:
                print("Please enter an angle between -88 and 88 degrees")
        except ValueError:
            print("Please enter a valid number")

def preview_angle_conversion(controller):
    """Show angle to steps conversion without sending"""
    angle = get_angle_input()
    steps = controller.angle_to_steps(angle)
    
    print(f"\nAngle Conversion Preview:")
    print(f"   Input angle: {angle}°")
    print(f"   Absolute angle: {abs(angle)}°")
    print(f"   Calculated steps: {steps}")
    print(f"   This would move the sail to approximately {abs(angle)}° from center")

def send_angle_command(controller):
    """Send angle command using tracked positioning"""
    angle = get_angle_input()
    return controller.move_to_angle(angle)

def quick_test(controller):
    """Run a quick test with 10 steps in each direction"""
    print("\nRunning quick test (10 steps each direction)...")
    
    print("Testing 10 CW steps (let out)...")
    controller.send_cw_steps(10)
    time.sleep(2)
    
    print("Testing 10 CCW steps (bring in)...")
    controller.send_ccw_steps(10)
    time.sleep(2)
    
    print("Quick test complete!")

def main():
    """Main program loop"""
    print("Starting Sail Winch Control Test Script...")
    print(f"Using sail parameters:")
    print(f"  Boom length: {BOOM_LENGTH} inches")
    print(f"  Winch to mast: {WINCH_TO_MAST} inches")
    print(f"  Spool radius: {SPOOL_RADIUS} inches")
    print(f"  Gear ratio: {GEAR_RATIO}")
    print(f"  Steps per revolution: {STEPS_PER_REVOLUTION}")
    
    # Initialize connection
    controller = ArduinoStepControl()
    
    while True:
        print_menu()
        
        try:
            choice = input("\nEnter your choice (1-8): ").strip()
            
            if choice == '1':
                send_angle_command(controller)
                
            elif choice == '2':
                controller.get_current_position()
                
            elif choice == '3':
                angle = get_angle_input()
                controller.set_current_position(angle)
                
            elif choice == '4':
                steps = get_step_input()
                controller.send_cw_steps(steps)
                
            elif choice == '5':
                steps = get_step_input()
                controller.send_ccw_steps(steps)
                
            elif choice == '6':
                preview_angle_conversion(controller)
                
            elif choice == '7':
                quick_test(controller)
                
            elif choice == '8':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice. Please enter 1-8.")
                
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
