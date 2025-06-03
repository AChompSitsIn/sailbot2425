#!/usr/bin/env python3
"""
winch_angle_test.py - Test script for winch angle control

This script replicates the angle logic from winch_control_node.py and allows
direct testing of angle-to-steps conversion and physical winch movement.

Usage:
    sudo python3 winch_angle_test.py

Enter sail angles in degrees (-88 to 88) to test the winch movement.
"""

import smbus
import time
import sys
import math

# I2C Configuration (from winch_control_node.py)
WINCH_ADDRESS = 0x08
I2C_BUS = 1

# Command registers (from winch_i2c.ino)
CMD_MOVE_STEPS = 1
CMD_SET_DIRECTION = 2
CMD_SET_ENABLE = 3
CMD_GET_POSITION = 4
CMD_RESET_POSITION = 5

# Winch parameters (from winch_control_node.py)
MAX_STEPS = 1600
MIN_SAIL_ANGLE = -88.0
MAX_SAIL_ANGLE = 88.0

# Physical parameters (from winch_control_node.py angle_to_steps method)
BOOM_LENGTH = 28        # inches
WINCH_TO_MAST = 40     # inches
SPOOL_RADIUS = 3       # inches
GEAR_RATIO = 10
STEPS_PER_REVOLUTION = 1600

bus = None
current_steps = 0  # Track current position

def send_i2c_command(bus_obj, address, command_reg, data_bytes):
    """Send command and data to I2C device."""
    try:
        bus_obj.write_i2c_block_data(address, command_reg, data_bytes)
        return True
    except IOError as e:
        print(f"I2C Error sending command {command_reg} to 0x{address:02X}: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

def read_i2c_data(bus_obj, address, num_bytes):
    """Read data from I2C device."""
    try:
        data = bus_obj.read_i2c_block_data(address, 0, num_bytes)
        return data
    except IOError as e:
        print(f"I2C Error reading from 0x{address:02X}: {e}")
        return None
    except Exception as e:
        print(f"Unexpected error reading: {e}")
        return None

def enable_motor(bus_obj, address, enable_flag):
    """Enable or disable the motor."""
    status = "ENABLED" if enable_flag else "DISABLED"
    print(f"Motor {status}")
    return send_i2c_command(bus_obj, address, CMD_SET_ENABLE, [1 if enable_flag else 0])

def set_direction(bus_obj, address, direction_flag):
    """Set motor direction (0 for CW, 1 for CCW)."""
    direction_name = "CCW" if direction_flag else "CW"
    print(f"Setting direction: {direction_name}")
    return send_i2c_command(bus_obj, address, CMD_SET_DIRECTION, [direction_flag])

def move_steps(bus_obj, address, steps):
    """Move specified number of steps."""
    if steps <= 0:
        print("Invalid step count")
        return False

    print(f"Moving {steps} steps")
    high_byte = (steps >> 8) & 0xFF
    low_byte = steps & 0xFF
    return send_i2c_command(bus_obj, address, CMD_MOVE_STEPS, [high_byte, low_byte])

def get_position(bus_obj, address):
    """Get current position from Arduino."""
    if send_i2c_command(bus_obj, address, CMD_GET_POSITION, []):
        time.sleep(0.1)  # Give Arduino time to prepare response
        data = read_i2c_data(bus_obj, address, 4)  # Read 4 bytes for int32
        if data:
            # Convert 4 bytes back to signed 32-bit integer
            position = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
            # Handle signed integer conversion
            if position >= 2**31:
                position -= 2**32
            return position
    return None

def reset_position(bus_obj, address):
    """Reset position counter to zero."""
    print("Resetting position counter to 0")
    return send_i2c_command(bus_obj, address, CMD_RESET_POSITION, [])

def angle_to_steps(target_angle_degrees: float) -> int:
    """
    Convert target sail angle in degrees to target motor steps.
    This is the EXACT same logic from winch_control_node.py
    """
    # Clamp the angle to ensure it's within the defined operational range
    target_angle_degrees = max(MIN_SAIL_ANGLE, min(MAX_SAIL_ANGLE, target_angle_degrees))

    sail_side = True  # True for port, False for starboard

    # keep track of if the sail side should be port or starboard
    if target_angle_degrees < 0:
        sail_side = False
    elif target_angle_degrees > 0:
        sail_side = True

    print(f"Sail side: {'Port' if sail_side else 'Starboard'}")

    target_angle_degrees = abs(target_angle_degrees)
    print(f"Absolute angle: {target_angle_degrees}")

    # Physical parameters from winch_control_node.py
    boom_length = BOOM_LENGTH
    winch_to_mast = WINCH_TO_MAST
    spool_radius = SPOOL_RADIUS
    gear_ratio = GEAR_RATIO
    steps_per_revolution = STEPS_PER_REVOLUTION

    # Calculate required cable length using law of cosines
    length = math.sqrt(boom_length**2 + winch_to_mast**2 -
                      2 * boom_length * winch_to_mast * math.cos(math.radians(target_angle_degrees)))

    print(f"Calculated cable length: {length:.2f} inches")

    # Convert to steps (same formula from winch_control_node.py)
    target_steps = int(length / (2 * math.pi * spool_radius * gear_ratio * steps_per_revolution * 2))

    print(f"Target steps: {target_steps}")
    return target_steps

def steps_to_angle(current_steps: int) -> float:
    """
    Convert current motor steps to estimated sail angle in degrees.
    This uses the simplified linear mapping from winch_control_node.py
    """
    if MAX_STEPS == 0:
        return MIN_SAIL_ANGLE

    normalized_position = float(current_steps) / float(MAX_STEPS)
    angle_range = MAX_SAIL_ANGLE - MIN_SAIL_ANGLE
    estimated_angle_degrees = MIN_SAIL_ANGLE + (normalized_position * angle_range)

    return estimated_angle_degrees

def move_to_angle(bus_obj, address, target_angle_degrees: float):
    """
    Move winch to specified angle using the same logic as winch_control_node.py
    """
    global current_steps

    print(f"\n{'='*50}")
    print(f"MOVING TO ANGLE: {target_angle_degrees}°")
    print(f"{'='*50}")

    # Convert target angle to target steps using winch_control_node logic
    target_steps = angle_to_steps(target_angle_degrees)

    # Calculate difference in steps
    steps_diff = target_steps - current_steps
    print(f"Current steps: {current_steps}")
    print(f"Target steps: {target_steps}")
    print(f"Steps difference: {steps_diff}")

    if steps_diff != 0:
        # Determine direction (same logic as winch_control_node.py)
        # 1 for CCW (typically increasing steps for sail trim in/positive angle)
        # 0 for CW (sail ease out/negative angle)
        direction = 1 if steps_diff > 0 else 0

        if not set_direction(bus_obj, address, direction):
            print("Failed to set winch direction.")
            return False

        # Send move command for the absolute number of steps
        steps_to_move = abs(steps_diff)

        print(f"Moving {steps_to_move} steps, direction: {'CCW (trim in/positive angle)' if direction == 1 else 'CW (ease out/negative angle)'}")

        if move_steps(bus_obj, address, steps_to_move):
            current_steps = target_steps
            # Calculate estimated new angle
            estimated_angle = steps_to_angle(current_steps)

            print(f"✅ Movement complete!")
            print(f"New current steps: {current_steps}")
            print(f"Estimated new angle: {estimated_angle:.1f}°")
            return True
        else:
            print(f"❌ Failed to move winch by {steps_to_move} steps.")
            return False
    else:
        print(f"✅ Winch already at target position for angle {target_angle_degrees:.1f}°. No movement needed.")
        estimated_angle = steps_to_angle(current_steps)
        print(f"Current estimated angle: {estimated_angle:.1f}°")
        return True

def print_menu():
    """Display the test menu."""
    print("\n" + "="*60)
    print("WINCH ANGLE TEST SCRIPT")
    print("="*60)
    print("This script tests the angle conversion logic from winch_control_node.py")
    print()
    print("Commands:")
    print("  <angle>  - Enter any angle in degrees (-88 to 88)")
    print("  p        - Show current position and estimated angle")
    print("  r        - Reset position to 0")
    print("  e        - Toggle motor enable/disable")
    print("  test     - Run a sequence of test angles")
    print("  m        - Show this menu")
    print("  q        - Quit")
    print()
    print("Physical Parameters:")
    print(f"  Boom length: {BOOM_LENGTH} inches")
    print(f"  Winch to mast: {WINCH_TO_MAST} inches")
    print(f"  Spool radius: {SPOOL_RADIUS} inches")
    print(f"  Gear ratio: {GEAR_RATIO}")
    print(f"  Steps per revolution: {STEPS_PER_REVOLUTION}")
    print("="*60)

def run_test_sequence(bus_obj, address):
    """Run a sequence of test angles."""
    test_angles = [0, 15, 30, 45, -15, -30, -45, 0]

    print(f"\n{'='*50}")
    print("RUNNING TEST SEQUENCE")
    print(f"{'='*50}")
    print(f"Test angles: {test_angles}")

    for angle in test_angles:
        print(f"\n--- Testing angle: {angle}° ---")
        if not move_to_angle(bus_obj, address, angle):
            print("Test sequence failed!")
            return

        # Wait between movements
        print("Waiting 2 seconds before next movement...")
        time.sleep(2)

    print(f"\n✅ Test sequence completed successfully!")

def main():
    global bus, current_steps

    try:
        # Initialize I2C
        bus = smbus.SMBus(I2C_BUS)
        print(f"I2C bus {I2C_BUS} initialized")
        print(f"Winch Arduino at address 0x{WINCH_ADDRESS:02X}")

        # Initial setup
        print("\nInitializing winch system...")

        # Enable motor
        if not enable_motor(bus, WINCH_ADDRESS, True):
            print("Failed to enable motor. Exiting.")
            sys.exit(1)

        time.sleep(0.5)

        # Reset position counter
        reset_position(bus, WINCH_ADDRESS)
        time.sleep(0.1)
        current_steps = 0

        # Show initial position
        pos = get_position(bus, WINCH_ADDRESS)
        if pos is not None:
            current_steps = pos
            print(f"Initial position: {current_steps} steps")

        print("\nWinch angle test system ready!")
        print_menu()

        motor_enabled = True

        while True:
            # Show current status
            pos = get_position(bus, WINCH_ADDRESS)
            if pos is not None:
                current_steps = pos

            estimated_angle = steps_to_angle(current_steps)
            status_line = f"Position: {current_steps} steps | Estimated Angle: {estimated_angle:.1f}° | Motor: {'ON' if motor_enabled else 'OFF'}"
            print(f"\n{status_line}")

            try:
                command = input("Enter command or angle: ").strip().lower()
            except KeyboardInterrupt:
                break

            if command == 'q':
                break
            elif command == 'm':
                print_menu()
            elif command == 'p':
                pos = get_position(bus, WINCH_ADDRESS)
                if pos is not None:
                    current_steps = pos
                est_angle = steps_to_angle(current_steps)
                print(f"Current position: {current_steps} steps")
                print(f"Estimated angle: {est_angle:.1f}°")
            elif command == 'r':
                reset_position(bus, WINCH_ADDRESS)
                time.sleep(0.1)
                current_steps = 0
                print("Position reset to 0")
            elif command == 'e':
                motor_enabled = not motor_enabled
                enable_motor(bus, WINCH_ADDRESS, motor_enabled)
            elif command == 'test':
                if motor_enabled:
                    run_test_sequence(bus, WINCH_ADDRESS)
                else:
                    print("Motor is disabled. Enable motor first with 'e' command.")
            else:
                # Try to parse as angle
                try:
                    target_angle = float(command)
                    if MIN_SAIL_ANGLE <= target_angle <= MAX_SAIL_ANGLE:
                        if motor_enabled:
                            move_to_angle(bus, WINCH_ADDRESS, target_angle)
                        else:
                            print("Motor is disabled. Enable motor first with 'e' command.")
                    else:
                        print(f"Angle out of range. Must be between {MIN_SAIL_ANGLE}° and {MAX_SAIL_ANGLE}°")
                except ValueError:
                    print("Invalid command or angle. Type 'm' for menu.")

            time.sleep(0.1)  # Small delay between commands

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if bus:
            print("\nShutting down...")
            enable_motor(bus, WINCH_ADDRESS, False)
            print("Motor disabled. Test complete.")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--help':
        print(__doc__)
        sys.exit(0)

    print("Winch Angle Test Script")
    print("Make sure to run with sudo for I2C access!")
    print("Press Ctrl+C to exit at any time.")
    main()
