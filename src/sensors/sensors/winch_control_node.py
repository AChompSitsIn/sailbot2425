#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import struct
import time
import math

class WinchControlNode(Node):
    """
    ROS2 Node for controlling the winch stepper motor via I2C

    Uses direct I2C communication with the winch Arduino (address 0x08)

    Subscribes to:
    - 'sail/angle': Target sail angle command in degrees (e.g., -90 to 90)

    Publishes:
    - 'sail/position': Current estimated sail angle in degrees
    """

    # Arduino I2C configuration
    WINCH_ADDRESS = 0x08
    I2C_BUS = 1  # Use I2C bus 1

    # Command registers from winch_i2c.ino
    CMD_MOVE_STEPS = 1  # Move a specified number of steps
    CMD_SET_DIRECTION = 2  # Set motor direction (0=CW, 1=CCW)
    CMD_SET_ENABLE = 3  # Enable/disable motor (0=disable, 1=enable)

    # Winch parameters
    MAX_STEPS = 1600  # Maximum number of steps (adjust as needed)

    def __init__(self):
        super().__init__('winch_control_node')

        # Declare and get parameters
        self.declare_parameter('i2c_bus', self.I2C_BUS)
        self.declare_parameter('winch_address', self.WINCH_ADDRESS)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('max_steps', self.MAX_STEPS)
        self.declare_parameter('min_sail_angle', -88.0) # Example: Minimum sail angle in degrees
        self.declare_parameter('max_sail_angle', 88.0)  # Example: Maximum sail angle in degrees

        # Get parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.winch_address = self.get_parameter('winch_address').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_steps = self.get_parameter('max_steps').value
        self.min_sail_angle = self.get_parameter('min_sail_angle').value
        self.max_sail_angle = self.get_parameter('max_sail_angle').value

        # Initialize state variables
        self.current_angle_degrees = 0.0  # Estimated current sail angle in degrees
        self.current_steps = 0 # Initialize current_steps, assuming a start position (0 steps = min pos = approx 88 deg)

        # Initialize publishers
        self.position_publisher = self.create_publisher( # This now publishes estimated sail angle
            Float32,
            'sail/position',
            10
        )

        # Initialize subscriber for sail control (sail angle)
        self.command_subscription = self.create_subscription(
            Float32,
            'sail/angle', # Changed from 'sail/control'
            self.command_callback,
            10
        )

        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            self.get_logger().info(
                f"I2C initialized on bus {self.i2c_bus}, "
                f"address 0x{self.winch_address:02X}"
            )

            # Enable the motor
            self.enable_motor(True)

        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")

        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_position)

        self.get_logger().info('Winch control node initialized')

    def send_command(self, command, *data):
        """
        Send a command to the winch Arduino

        Args:
            command: Command code
            *data: Command parameters
        """
        try:
            # Command-specific formatting
            if command == self.CMD_MOVE_STEPS and len(data) == 1:
                steps = data[0]
                high_byte = (steps >> 8) & 0xFF
                low_byte = steps & 0xFF

                self.bus.write_i2c_block_data(
                    self.winch_address,
                    command,
                    [high_byte, low_byte]
                )
            else:
                # For simple commands like direction or enable
                self.bus.write_i2c_block_data(
                    self.winch_address,
                    command,
                    list(data)
                )

            return True
        except Exception as e:
            self.get_logger().error(f"Error sending command to winch Arduino: {e}")
            return False

    def enable_motor(self, enable):
        """
        Enable or disable the stepper motor

        Args:
            enable: True to enable, False to disable
        """
        value = 1 if enable else 0
        result = self.send_command(self.CMD_SET_ENABLE, value)
        if result:
            status = "enabled" if enable else "disabled"
            self.get_logger().info(f"Winch motor {status}")

        return result

    def set_direction(self, direction):
        """
        Set the stepper motor direction

        Args:
            direction: 0 for CW, 1 for CCW
        """
        return self.send_command(self.CMD_SET_DIRECTION, direction)

    def move_steps(self, steps):
        """
        Move the stepper motor a specified number of steps

        Args:
            steps: Number of steps to move
        """
        return self.send_command(self.CMD_MOVE_STEPS, steps)

    def angle_to_steps(self, target_angle_degrees: float) -> int:
        """
        Convert target sail angle in degrees to target motor steps.

        Args:
            target_angle_degrees: The desired sail angle in degrees.
                                 (e.g., -90 for full port, 0 for centered, 90 for full starboard)

        Returns:
            The corresponding number of motor steps from the zero/reference position.
        """
        # Clamp the angle to ensure it's within the defined operational range
        target_angle_degrees = max(self.min_sail_angle, min(self.max_sail_angle, target_angle_degrees))

        sail_side = True # True for port, False for starboard

        # keep track of if the sail side should be port or starboard
        if target_angle_degrees < 0:
            sail_side = False
        elif target_angle_degrees > 0:
            sail_side = True

        print(sail_side)

        target_angle_degrees = abs(target_angle_degrees)

        print(target_angle_degrees)

        boom_length = 28 # inches
        winch_to_mast = 40 # inches
        spool_radius = 3 # inches
        gear_ratio = 10
        steps_per_revolution = 1600

        length = math.sqrt(boom_length**2 + winch_to_mast**2 - 2 * boom_length * winch_to_mast * math.cos(math.radians(target_angle_degrees)))
        target_steps = int(length / (2 * math.pi * spool_radius * gear_ratio * steps_per_revolution * 2)) # the *2 is for the conversion for the other gear ratio

        return target_steps

    def steps_to_angle(self, current_steps: int) -> float:
        """
        Convert current motor steps to estimated sail angle in degrees.

        Args:
            current_steps: The current number of motor steps from the zero/reference position.

        Returns:
            The estimated sail angle in degrees.
        """
        # ####################################################################
        # START OF USER IMPLEMENTATION AREA: STEPS TO ANGLE CONVERSION
        # ####################################################################
        #
        # Implement your logic here to convert `current_steps` back to `estimated_angle_degrees`.
        # This is the inverse of the `angle_to_steps` logic.
        #
        # Example (LINEAR MAPPING, INVERSE OF THE ABOVE `angle_to_steps` EXAMPLE):
        # if self.max_steps == 0: # Avoid division by zero
        #    return self.min_sail_angle # Or some default
        # normalized_position = float(current_steps) / float(self.max_steps)
        # range_of_angles = self.max_sail_angle - self.min_sail_angle
        # estimated_angle_degrees = self.min_sail_angle + (normalized_position * range_of_angles)

        # Placeholder: Inverse of the placeholder in angle_to_steps
        if self.max_steps == 0:
            return self.min_sail_angle # Or some default like 0.0

        normalized_position = float(current_steps) / float(self.max_steps)
        angle_range = self.max_sail_angle - self.min_sail_angle
        estimated_angle_degrees = self.min_sail_angle + (normalized_position * angle_range)

        self.get_logger().debug(f"Steps {current_steps} -> Normalized {normalized_position:.2f} -> Estimated Angle: {estimated_angle_degrees:.1f}deg")
        # ####################################################################
        # END OF USER IMPLEMENTATION AREA
        # ####################################################################

        return estimated_angle_degrees

    def command_callback(self, msg: Float32):
        """
        Handle sail angle commands.

        Args:
            msg: Float32 message with target sail angle in degrees.
        """
        target_angle_degrees = msg.data

        # Log the received command
        self.get_logger().info(f"Received sail angle command: {target_angle_degrees:.1f} degrees")

        # Convert target angle to target steps using user-defined logic
        target_steps = self.angle_to_steps(target_angle_degrees)

        # Calculate difference in steps
        steps_diff = target_steps - self.current_steps

        if steps_diff != 0:
            # Determine direction: 1 for CCW (typically increasing steps for sail trim in/positive angle), 0 for CW (sail ease out/negative angle)
            # This depends on your winch setup and how you define steps vs angle.
            # If positive angle (e.g., starboard trim) means more steps:
            direction = 1 if steps_diff > 0 else 0
            # If positive angle means fewer steps (e.g., steps count down from max for starboard trim):
            # direction = 0 if steps_diff > 0 else 1

            if not self.set_direction(direction):
                self.get_logger().error("Failed to set winch direction.")
                return

            # Send move command for the absolute number of steps
            steps_to_move = abs(steps_diff)
            if self.move_steps(steps_to_move):
                self.current_steps = target_steps
                # Update current_angle_degrees based on the new step count
                self.current_angle_degrees = self.steps_to_angle(self.current_steps)
                self.get_logger().info(
                    f"Moving sail to target angle {target_angle_degrees:.1f}° (target steps: {target_steps}). "
                    f"Moving {steps_to_move} steps, direction: {'CCW (trim in/positive angle)' if direction == 1 else 'CW (ease out/negative angle)'}. "
                    f"New current steps: {self.current_steps}, Estimated new angle: {self.current_angle_degrees:.1f}°"
                )

                # Publish updated estimated sail angle
                self.publish_position()
            else:
                self.get_logger().error(f"Failed to move winch by {steps_to_move} steps.")
        else:
            self.get_logger().info(f"Sail already at target steps ({target_steps}) for angle {target_angle_degrees:.1f}°. No movement needed.")
            # Ensure the current angle is also up-to-date if no movement
            self.current_angle_degrees = self.steps_to_angle(self.current_steps)
            self.publish_position()

    def update_position(self):
        """Periodic update - publish current estimated sail angle"""
        # Potentially, you could add a read from the Arduino if it can report its actual step count.
        # For now, we rely on our internal tracking.
        self.publish_position()

    def publish_position(self):
        """Publish current estimated sail angle in degrees"""
        msg = Float32()
        # Convert current_steps back to an angle for publishing
        self.current_angle_degrees = self.steps_to_angle(self.current_steps)
        msg.data = float(self.current_angle_degrees)
        self.position_publisher.publish(msg)
        self.get_logger().debug(f"Published current estimated sail angle: {self.current_angle_degrees:.1f}° (from {self.current_steps} steps)")

    def destroy_node(self):
        """Clean up on shutdown"""
        try:
            # Disable motor
            self.enable_motor(False)
            self.get_logger().info("Disabled winch motor on shutdown")
        except Exception as e:
            self.get_logger().error(f"Error on shutdown: {e}")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = WinchControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
