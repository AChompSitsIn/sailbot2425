#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import time

class RudderControlNode(Node):
    """
    ROS2 Node for controlling the rudder servo via I2C
    
    Uses direct I2C communication with the rudder Arduino (address 0x09)
    
    Subscribes to:
    - 'rudder/command': Angle in degrees (-21 to 21)
    
    Publishes:
    - 'rudder/position': Current position of the rudder (servo angle 36-78)
    """
    
    # Arduino I2C configuration
    RUDDER_ADDRESS = 0x09
    I2C_BUS = 1  # Use I2C bus 1
    SERVO_CMD = 0x20  # Command byte for servo control
    
    # Rudder configuration based on new system
    NEUTRAL_SERVO_ANGLE = 57  # True 0 position for rudder
    MIN_RUDDER_ANGLE = -21.0  # Maximum angle to port (left)
    MAX_RUDDER_ANGLE = 21.0   # Maximum angle to starboard (right)
    
    # Corresponding servo positions
    MIN_SERVO_POSITION = NEUTRAL_SERVO_ANGLE + MIN_RUDDER_ANGLE  # 36
    MAX_SERVO_POSITION = NEUTRAL_SERVO_ANGLE + MAX_RUDDER_ANGLE  # 78
    
    def __init__(self):
        super().__init__('rudder_control_node')
        
        # Declare and get parameters
        self.declare_parameter('i2c_bus', self.I2C_BUS)
        self.declare_parameter('rudder_address', self.RUDDER_ADDRESS)
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.rudder_address = self.get_parameter('rudder_address').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize state variables
        self.current_rudder_angle = 0.0  # Current rudder angle (-21 to +21)
        self.current_servo_position = self.NEUTRAL_SERVO_ANGLE  # Current servo position
        self.target_rudder_angle = 0.0
        
        # Initialize publishers
        self.position_publisher = self.create_publisher(
            Float32,
            'rudder/position',
            10
        )
        
        # Initialize subscriber for rudder commands
        self.command_subscription = self.create_subscription(
            Float32,
            'rudder/command',
            self.command_callback,
            10
        )
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            self.get_logger().info(
                f"I2C initialized on bus {self.i2c_bus}, "
                f"address 0x{self.rudder_address:02X}"
            )
            
            # Set initial position to neutral (0°)
            self.set_rudder_angle(0.0)
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_rudder)
        
        self.get_logger().info(
            f'Rudder control node initialized - Range: {self.MIN_RUDDER_ANGLE}° to {self.MAX_RUDDER_ANGLE}°, '
            f'Servo range: {self.MIN_SERVO_POSITION}-{self.MAX_SERVO_POSITION}'
        )
    
    def rudder_angle_to_servo_position(self, rudder_angle):
        """
        Convert from rudder angle (-21 to 21) to servo position
        
        Uses formula: servo_position = 57 + rudder_angle
        
        Args:
            rudder_angle: Rudder angle in degrees (-21 to 21)
                         Negative = left/port, Positive = right/starboard
            
        Returns:
            Servo position value (36 to 78)
        """
        # Constrain rudder angle to safe range
        rudder_angle = max(self.MIN_RUDDER_ANGLE, min(self.MAX_RUDDER_ANGLE, rudder_angle))
        
        # Convert using the formula: servo_angle = 57 + rudder_angle
        servo_position = int(self.NEUTRAL_SERVO_ANGLE + rudder_angle)
        
        return servo_position
    
    def servo_position_to_rudder_angle(self, servo_position):
        """
        Convert from servo position back to rudder angle
        
        Args:
            servo_position: Servo position (36 to 78)
            
        Returns:
            Rudder angle in degrees (-21 to 21)
        """
        return float(servo_position - self.NEUTRAL_SERVO_ANGLE)
    
    def set_rudder_angle(self, rudder_angle):
        """
        Set rudder to specified angle via I2C
        
        Args:
            rudder_angle: Rudder angle in degrees (-21 to 21)
                         Negative = left/port, Positive = right/starboard
        """
        # Convert to servo position
        servo_position = self.rudder_angle_to_servo_position(rudder_angle)
        
        try:
            # Send command using block data with command byte
            self.bus.write_i2c_block_data(
                self.rudder_address, 
                self.SERVO_CMD, 
                [servo_position]
            )
            
            # Update current state
            self.current_servo_position = servo_position
            self.current_rudder_angle = self.servo_position_to_rudder_angle(servo_position)
            
            self.get_logger().debug(
                f"Rudder angle: {self.current_rudder_angle:+.1f}° → "
                f"Servo position: {servo_position}°"
            )
            
            # Publish current position
            self.publish_position()
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error setting rudder angle: {e}")
            return False
    
    def command_callback(self, msg):
        """
        Handle rudder command messages (angles)
        
        Args:
            msg: Float32 message with rudder angle in degrees (-21 to 21)
        """
        rudder_angle = msg.data
        
        # Validate input range
        if rudder_angle < self.MIN_RUDDER_ANGLE or rudder_angle > self.MAX_RUDDER_ANGLE:
            self.get_logger().warn(
                f"Received rudder angle {rudder_angle:.1f}° is outside valid range "
                f"({self.MIN_RUDDER_ANGLE}° to {self.MAX_RUDDER_ANGLE}°). Clamping."
            )
        
        servo_position = self.rudder_angle_to_servo_position(rudder_angle)
        
        self.get_logger().info(
            f"Received rudder command: {rudder_angle:+.1f}° → servo position: {servo_position}°"
        )
        
        self.target_rudder_angle = rudder_angle
    
    def update_rudder(self):
        """Periodic update - set rudder to target angle if needed"""
        if abs(self.target_rudder_angle - self.current_rudder_angle) > 0.1:  # 0.1° tolerance
            self.set_rudder_angle(self.target_rudder_angle)
    
    def publish_position(self):
        """Publish current rudder angle"""
        msg = Float32()
        msg.data = float(self.current_rudder_angle)
        self.position_publisher.publish(msg)
    
    def destroy_node(self):
        """Clean up and center rudder on shutdown"""
        try:
            # Return to neutral position (0°)
            self.set_rudder_angle(0.0)
            time.sleep(0.5)  # Give it time to center
            self.get_logger().info("Centered rudder on shutdown")
        except Exception as e:
            self.get_logger().error(f"Error on shutdown: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RudderControlNode()
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
