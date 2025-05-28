#!/usr/bin/env python3
import smbus
import time
import sys

# I2C Configuration (from WinchControlNode)
WINCH_ADDRESS = 0x08  # Default I2C address for the winch Arduino
I2C_BUS = 1           # Default I2C bus on Raspberry Pi / Jetson

# Command registers (from winch_i2c.ino via WinchControlNode)
CMD_MOVE_STEPS = 1    # Move a specified number of steps
CMD_SET_DIRECTION = 2 # Set motor direction (0=CW, 1=CCW)
CMD_SET_ENABLE = 3    # Enable/disable motor (0=disable, 1=enable)

# Loop parameters
STEPS_TO_MOVE = 400  # Number of steps to move in each direction (adjust as needed)
DELAY_SECONDS = 1.0   # Delay after each movement

bus = None

def send_i2c_command(bus_obj, address, command_reg, data_bytes):
    """
    Sends a command and data to the I2C device.
    Args:
        bus_obj: The smbus object.
        address: The I2C address of the slave device.
        command_reg: The command register to write to.
        data_bytes: A list of data bytes to send.
    """
    try:
        bus_obj.write_i2c_block_data(address, command_reg, data_bytes)
        # print(f"Sent command {command_reg} with data {data_bytes} to 0x{address:02X}")
        return True
    except IOError as e:
        print(f"I2C Error sending command {command_reg} to 0x{address:02X}: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return False

def enable_motor(bus_obj, address, enable_flag):
    """Enable or disable the motor."""
    print(f"Setting motor enable to: {enable_flag}")
    return send_i2c_command(bus_obj, address, CMD_SET_ENABLE, [1 if enable_flag else 0])

def set_direction(bus_obj, address, direction_flag):
    """Set motor direction (0 for CW, 1 for CCW)."""
    print(f"Setting direction to: {'CCW' if direction_flag else 'CW'}")
    return send_i2c_command(bus_obj, address, CMD_SET_DIRECTION, [direction_flag])

def move_steps(bus_obj, address, steps):
    """Command the motor to move a specific number of steps."""
    print(f"Moving {steps} steps.")
    high_byte = (steps >> 8) & 0xFF
    low_byte = steps & 0xFF
    return send_i2c_command(bus_obj, address, CMD_MOVE_STEPS, [high_byte, low_byte])

def main():
    global bus
    try:
        # Initialize I2C bus
        bus = smbus.SMBus(I2C_BUS)
        print(f"I2C bus {I2C_BUS} initialized.")
        print(f"Winch Arduino expected at address 0x{WINCH_ADDRESS:02X}.")

        # Enable the motor
        if not enable_motor(bus, WINCH_ADDRESS, True):
            print("Failed to enable motor. Exiting.")
            sys.exit(1)

        time.sleep(0.5) # Give a moment for the motor to enable

        print(f"Starting winch loop: CW/CCW {STEPS_TO_MOVE} steps with {DELAY_SECONDS}s delay.")
        print("Press Ctrl+C to stop.")

        while True:
            # Move Clockwise (Direction 0)
            if not set_direction(bus, WINCH_ADDRESS, 0): # 0 for CW
                print("Failed to set direction CW. Exiting loop.")
                break
            time.sleep(0.1) # Short delay after setting direction
            if not move_steps(bus, WINCH_ADDRESS, STEPS_TO_MOVE):
                print("Failed to move steps CW. Exiting loop.")
                break
            time.sleep(DELAY_SECONDS)

            # Move Counter-Clockwise (Direction 1)
            if not set_direction(bus, WINCH_ADDRESS, 1): # 1 for CCW
                print("Failed to set direction CCW. Exiting loop.")
                break
            time.sleep(0.1)
            if not move_steps(bus, WINCH_ADDRESS, STEPS_TO_MOVE):
                print("Failed to move steps CCW. Exiting loop.")
                break
            time.sleep(DELAY_SECONDS)

    except KeyboardInterrupt:
        print("\nLoop interrupted by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if bus:
            print("Disabling motor...")
            enable_motor(bus, WINCH_ADDRESS, False) # Disable the motor
            print("Motor disabled. Exiting.")

if __name__ == '__main__':
    main()
