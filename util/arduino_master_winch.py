import smbus
import time

# I2C settings
I2C_BUS = 1           # Use I2C bus 1 on Jetson
I2C_ADDRESS = 0x08    # Arduino slave address

# Command registers
CMD_MOVE_STEPS = 1    # Move a specified number of steps
CMD_SET_DIRECTION = 2 # Set motor direction (0=CW, 1=CCW)
CMD_SET_ENABLE = 3    # Enable/disable motor (0=disable, 1=enable)

# Direction constants
DIR_CW = 0
DIR_CCW = 1

class StepperController:
    def __init__(self):
        # Initialize I2C bus
        self.bus = smbus.SMBus(I2C_BUS)
        print(f"Initialized I2C bus {I2C_BUS} for stepper control")
    
    def enable_motor(self, enable=True):
        """Enable or disable the motor"""
        try:
            self.bus.write_byte_data(I2C_ADDRESS, CMD_SET_ENABLE, 1 if enable else 0)
            print(f"Motor {'enabled' if enable else 'disabled'}")
            time.sleep(0.1)  # Give Arduino time to process
            return True
        except Exception as e:
            print(f"Error enabling motor: {e}")
            return False
    
    def set_direction(self, direction):
        """Set motor direction (0=CW, 1=CCW)"""
        try:
            self.bus.write_byte_data(I2C_ADDRESS, CMD_SET_DIRECTION, 1 if direction else 0)
            print(f"Direction set to {'CCW' if direction else 'CW'}")
            time.sleep(0.1)  # Give Arduino time to process
            return True
        except Exception as e:
            print(f"Error setting direction: {e}")
            return False
    
    def move_steps(self, steps):
        """Move motor by specified number of steps"""
        if steps > 65535:
            print("Steps value exceeds maximum (65535)")
            return False
        
        try:
            # Split steps into high and low bytes
            high_byte = (steps >> 8) & 0xFF
            low_byte = steps & 0xFF
            
            # Send command with step count
            self.bus.write_i2c_block_data(I2C_ADDRESS, CMD_MOVE_STEPS, [high_byte, low_byte])
            print(f"Sent move command: {steps} steps")
            return True
        except Exception as e:
            print(f"Error sending move command: {e}")
            return False
    
    def rotate_revolutions(self, revolutions, direction=DIR_CW):
        """Rotate motor by specified number of revolutions"""
        STEPS_PER_REV = 1600  # Must match Arduino setting
        steps = int(revolutions * STEPS_PER_REV)
        
        # Set direction first
        if not self.set_direction(direction):
            return False
        
        # Then move steps
        return self.move_steps(steps)

# Example usage
if __name__ == "__main__":
    controller = StepperController()
    
    # Enable the motor
    controller.enable_motor(True)
    time.sleep(1)
    
    # Rotate 1 revolution clockwise
    print("Rotating 1 revolution clockwise")
    controller.rotate_revolutions(1, DIR_CW)
    time.sleep(3)  # Wait for movement to complete
    
    # Rotate 1 revolution counter-clockwise
    print("Rotating 1 revolution counter-clockwise")
    controller.rotate_revolutions(1, DIR_CCW)
    time.sleep(3)  # Wait for movement to complete
    
    # Disable the motor when done
    controller.enable_motor(False)
    
    print("Motor control demonstration complete")