#!/usr/bin/env python3
import smbus
import time
import sys

# I2C Configuration
WINCH_ADDRESS = 0x08
I2C_BUS = 1

# Command registers
CMD_MOVE_STEPS = 1
CMD_SET_DIRECTION = 2
CMD_SET_ENABLE = 3
CMD_GET_POSITION = 4  # New command to get current position
CMD_RESET_POSITION = 5  # New command to reset position counter

# Calibration parameters
DEFAULT_STEP_SIZE = 50      # Default small step size for calibration
FAST_STEP_SIZE = 200        # Larger step size for faster movement
SLOW_STEP_SIZE = 10         # Very small steps for fine positioning

bus = None
current_direction = 0  # 0 = CW, 1 = CCW

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
    global current_direction
    current_direction = direction_flag
    direction_name = "CCW" if direction_flag else "CW"
    print(f"Direction: {direction_name}")
    return send_i2c_command(bus_obj, address, CMD_SET_DIRECTION, [direction_flag])

def move_steps(bus_obj, address, steps):
    """Move specified number of steps."""
    if steps <= 0:
        print("Invalid step count")
        return False

    direction_name = "CCW" if current_direction else "CW"
    print(f"Moving {steps} steps {direction_name}")

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

def print_menu():
    """Display the calibration menu."""
    print("\n" + "="*50)
    print("WINCH CALIBRATION MENU")
    print("="*50)
    print("Movement Commands:")
    print("  1 - Move CW (small steps)")
    print("  2 - Move CCW (small steps)")
    print("  3 - Move CW (fast)")
    print("  4 - Move CCW (fast)")
    print("  5 - Move CW (fine)")
    print("  6 - Move CCW (fine)")
    print("  c - Custom step size")
    print()
    print("Position Commands:")
    print("  p - Show current position")
    print("  r - Reset position to 0")
    print()
    print("System Commands:")
    print("  e - Toggle motor enable/disable")
    print("  m - Show this menu")
    print("  q - Quit")
    print("="*50)

def get_user_input():
    """Get and validate user input."""
    try:
        return input("Enter command: ").strip().lower()
    except KeyboardInterrupt:
        return 'q'

def main():
    global bus

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

        # Show initial position
        pos = get_position(bus, WINCH_ADDRESS)
        if pos is not None:
            print(f"Initial position: {pos} steps")

        print("\nWinch calibration system ready!")
        print("Use this tool to find your winch's physical limits.")
        print("Start with small movements and observe the physical system.")

        print_menu()

        motor_enabled = True

        while True:
            # Show current status
            pos = get_position(bus, WINCH_ADDRESS)
            status_line = f"Position: {pos if pos is not None else 'Unknown'} | Motor: {'ON' if motor_enabled else 'OFF'}"
            print(f"\n{status_line}")

            command = get_user_input()

            if command == 'q':
                break
            elif command == 'm':
                print_menu()
            elif command == 'p':
                pos = get_position(bus, WINCH_ADDRESS)
                print(f"Current position: {pos if pos is not None else 'Unable to read'} steps")
            elif command == 'r':
                reset_position(bus, WINCH_ADDRESS)
                time.sleep(0.1)
            elif command == 'e':
                motor_enabled = not motor_enabled
                enable_motor(bus, WINCH_ADDRESS, motor_enabled)
            elif command == '1':  # CW small
                set_direction(bus, WINCH_ADDRESS, 0)
                move_steps(bus, WINCH_ADDRESS, DEFAULT_STEP_SIZE)
            elif command == '2':  # CCW small
                set_direction(bus, WINCH_ADDRESS, 1)
                move_steps(bus, WINCH_ADDRESS, DEFAULT_STEP_SIZE)
            elif command == '3':  # CW fast
                set_direction(bus, WINCH_ADDRESS, 0)
                move_steps(bus, WINCH_ADDRESS, FAST_STEP_SIZE)
            elif command == '4':  # CCW fast
                set_direction(bus, WINCH_ADDRESS, 1)
                move_steps(bus, WINCH_ADDRESS, FAST_STEP_SIZE)
            elif command == '5':  # CW fine
                set_direction(bus, WINCH_ADDRESS, 0)
                move_steps(bus, WINCH_ADDRESS, SLOW_STEP_SIZE)
            elif command == '6':  # CCW fine
                set_direction(bus, WINCH_ADDRESS, 1)
                move_steps(bus, WINCH_ADDRESS, SLOW_STEP_SIZE)
            elif command == 'c':  # Custom steps
                try:
                    steps = int(input("Enter number of steps: "))
                    direction = input("Enter direction (cw/ccw): ").strip().lower()
                    if direction in ['cw', 'ccw']:
                        dir_flag = 0 if direction == 'cw' else 1
                        set_direction(bus, WINCH_ADDRESS, dir_flag)
                        move_steps(bus, WINCH_ADDRESS, abs(steps))
                    else:
                        print("Invalid direction. Use 'cw' or 'ccw'")
                except ValueError:
                    print("Invalid step count. Enter a number.")
            else:
                print("Invalid command. Type 'm' for menu.")

            time.sleep(0.1)  # Small delay between commands

    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if bus:
            print("\nShutting down...")
            enable_motor(bus, WINCH_ADDRESS, False)
            print("Motor disabled. Calibration complete.")

if __name__ == '__main__':
    main()
