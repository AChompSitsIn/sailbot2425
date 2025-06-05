#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import time
import math

class WinchControlNode(Node):
    """
    ROS2 Node for controlling the winch stepper motor via I2C

    Uses direct I2C communication with the winch Arduino (address 0x08)

    Subscribes to:
    - 'sail/angle': Target sail angle command in degrees (e.g., -88 to 88)

    Publishes:
    - 'sail/position': Current estimated sail angle in degrees
    """

    # Arduino I2C configuration
    WINCH_ADDRESS = 0x08
    I2C_BUS = 1  # Use I2C bus 1

    # Command codes (matching the standalone script)
    CMD_WINCH_CW_STEPS = 0x12   # Clockwise - let sail out
    CMD_WINCH_CCW_STEPS = 0x13  # Counter-clockwise - bring sail in

    # Sail angle conversion parameters (updated values)
    BOOM_LENGTH = 48          # inches
    WINCH_TO_MAST = 44        # inches
    SPOOL_RADIUS = 1.5        # inches
    GEAR_RATIO = 5            # Changed from 10 to 5
    STEPS_PER_REVOLUTION = 1600

    # Sail angle limits
    MIN_SAIL_ANGLE = -88.0
    MAX_SAIL_ANGLE = 88.0

    def __init__(self):
        super().__init__('winch_control_node')

        # Declare and get parameters
        self.declare_parameter('i2c_bus', self.I2C_BUS)
        self.declare_parameter('winch_address', self.WINCH_ADDRESS)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('boom_length', self.BOOM_LENGTH)
        self.declare_parameter('winch_to_mast', self.WINCH_TO_MAST)
        self.declare_parameter('spool_radius', self.SPOOL_RADIUS)
        self.declare_parameter('gear_ratio', self.GEAR_RATIO)
        self.declare_parameter('steps_per_revolution', self.STEPS_PER_REVOLUTION)

        # Get parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.winch_address = self.get_parameter('winch_address').value
        self.update_rate = self.get_parameter('update_rate').value
        self.boom_length = self.get_parameter('boom_length').value
        self.winch_to_mast = self.get_parameter('winch_to_mast').value
        self.spool_radius = self.get_parameter('spool_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.steps_per_revolution = self.get_parameter('steps_per_revolution').value

        # Initialize state variables (track current position like standalone script)
        self.current_angle = 0.0      # Current sail angle in degrees
        self.current_steps = 0        # Current step position
        self.target_angle = 0.0       # Target sail angle

        # Initialize publishers
        self.position_publisher = self.create_publisher(
            Float32,
            'sail/position',
            10
        )

        # Initialize subscriber for sail control
        self.command_subscription = self.create_subscription(
            Float32,
            'sail/angle',
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
            
            # Log sail parameters
            self.get_logger().info(
                f"Sail parameters - Boom: {self.boom_length}in, "
                f"Winch-to-mast: {self.winch_to_mast}in, "
                f"Spool radius: {self.spool_radius}in, "
                f"Gear ratio: {self.gear_ratio}, "
                f"Steps/rev: {self.steps_per_revolution}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")

        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_position)

        self.get_logger().info('Winch control node initialized')
        self.get_logger().info(f'Initial sail position: {self.current_angle}° ({self.current_steps} steps)')

    def angle_to_steps(self, target_angle_degrees):
        """
        Convert target sail angle in degrees to motor steps using winch math.
        Uses law of cosines like the standalone script.
        
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
            self.boom_length**2 + self.winch_to_mast**2 - 
            2 * self.boom_length * self.winch_to_mast * math.cos(math.radians(target_angle_degrees))
        )
        
        # Convert to steps: length -> spool rotations -> motor rotations -> steps
        spool_circumference = 2 * math.pi * self.spool_radius
        spool_rotations = length / spool_circumference
        motor_rotations = spool_rotations * self.gear_ratio
        target_steps = int(motor_rotations * self.steps_per_revolution)
        
        return target_steps

    def steps_to_angle(self, current_steps):
        """
        Convert current motor steps back to estimated sail angle.
        This is an approximation for position reporting.
        
        Args:
            current_steps: The current number of motor steps
            
        Returns:
            The estimated sail angle in degrees
        """
        # Reverse the angle_to_steps calculation
        # This is approximate since we're doing inverse trig
        
        if current_steps <= 0:
            return 0.0
        
        # Calculate length from steps
        motor_rotations = current_steps / self.steps_per_revolution
        spool_rotations = motor_rotations / self.gear_ratio
        spool_circumference = 2 * math.pi * self.spool_radius
        length = spool_rotations * spool_circumference
        
        # Use law of cosines to find angle
        # length^2 = boom^2 + winch_to_mast^2 - 2*boom*winch_to_mast*cos(angle)
        # Solve for angle:
        cos_angle = (self.boom_length**2 + self.winch_to_mast**2 - length**2) / (2 * self.boom_length * self.winch_to_mast)
        
        # Clamp cos_angle to valid range [-1, 1]
        cos_angle = max(-1.0, min(1.0, cos_angle))
        
        angle_radians = math.acos(cos_angle)
        angle_degrees = math.degrees(angle_radians)
        
        return max(0.0, min(88.0, angle_degrees))

    def send_cw_steps(self, steps):
        """Send clockwise steps command (let sail out)"""
        if not (0 <= steps <= 100000):
            self.get_logger().error(f"Steps must be between 0 and 100,000 (got {steps})")
            return False
        
        try:
            # Prepare command: 1 byte command + 4 bytes step count (big-endian)
            data = [(steps >> 24) & 0xFF, (steps >> 16) & 0xFF, 
                   (steps >> 8) & 0xFF, steps & 0xFF]
            
            self.bus.write_i2c_block_data(self.winch_address, self.CMD_WINCH_CW_STEPS, data)
            self.get_logger().debug(f"Sent: CW {steps} steps (let sail out)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending CW command: {e}")
            return False

    def send_ccw_steps(self, steps):
        """Send counter-clockwise steps command (bring sail in)"""
        if not (0 <= steps <= 100000):
            self.get_logger().error(f"Steps must be between 0 and 100,000 (got {steps})")
            return False
        
        try:
            # Prepare command: 1 byte command + 4 bytes step count (big-endian)
            data = [(steps >> 24) & 0xFF, (steps >> 16) & 0xFF, 
                   (steps >> 8) & 0xFF, steps & 0xFF]
            
            self.bus.write_i2c_block_data(self.winch_address, self.CMD_WINCH_CCW_STEPS, data)
            self.get_logger().debug(f"Sent: CCW {steps} steps (bring sail in)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending CCW command: {e}")
            return False

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
        
        self.get_logger().info(
            f"Moving sail from {self.current_angle:.1f}° to {target_angle:.1f}°"
        )
        self.get_logger().debug(
            f"Current steps: {self.current_steps}, Target steps: {target_steps}, "
            f"Steps difference: {steps_difference}"
        )
        
        if steps_difference == 0:
            self.get_logger().info("Sail already at target position!")
            return True
        
        # Determine direction and send command
        success = False
        if steps_difference > 0:
            # Need to let sail out (CW)
            self.get_logger().info(f"Sending CW {abs(steps_difference)} steps (let out)")
            success = self.send_cw_steps(abs(steps_difference))
        else:
            # Need to bring sail in (CCW) 
            self.get_logger().info(f"Sending CCW {abs(steps_difference)} steps (bring in)")
            success = self.send_ccw_steps(abs(steps_difference))
        
        # Update tracked position if successful
        if success:
            self.current_angle = target_angle
            self.current_steps = target_steps
            self.get_logger().info(
                f"Updated sail position: {self.current_angle:.1f}° ({self.current_steps} steps)"
            )
            # Publish updated position
            self.publish_position()
        
        return success

    def set_current_position(self, angle):
        """Set the current sail position (for calibration)"""
        angle = abs(angle)
        angle = max(0.0, min(88.0, angle))
        
        self.current_angle = angle
        self.current_steps = self.angle_to_steps(angle)
        self.get_logger().info(f"Set current sail position: {self.current_angle:.1f}° ({self.current_steps} steps)")
        self.publish_position()

    def command_callback(self, msg):
        """
        Handle sail angle commands.

        Args:
            msg: Float32 message with target sail angle in degrees.
        """
        target_angle = msg.data

        # Log the received command
        self.get_logger().info(f"Received sail angle command: {target_angle:.1f}°")

        # Validate range
        if abs(target_angle) > 88.0:
            self.get_logger().warn(
                f"Target angle {target_angle:.1f}° exceeds maximum (±88°). Clamping."
            )

        self.target_angle = target_angle

    def update_position(self):
        """Periodic update - move to target angle if needed and publish position"""
        # Check if we need to move to target
        if abs(self.target_angle) != self.current_angle:
            self.move_to_angle(self.target_angle)
        else:
            # Just publish current position
            self.publish_position()

    def publish_position(self):
        """Publish current estimated sail angle in degrees"""
        msg = Float32()
        msg.data = float(self.current_angle)
        self.position_publisher.publish(msg)
        self.get_logger().debug(f"Published current sail angle: {self.current_angle:.1f}°")

    def destroy_node(self):
        """Clean up on shutdown"""
        try:
            self.get_logger().info("Winch control node shutting down")
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
