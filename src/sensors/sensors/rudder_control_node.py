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
    - 'rudder/command': Angle in degrees (-45 to 45)
    
    Publishes:
    - 'rudder/position': Current position of the rudder (arduino servo value 30-160)
    """
    
    # Arduino I2C configuration
    RUDDER_ADDRESS = 0x09
    I2C_BUS = 1  # Use I2C bus 1
    
    # Rudder position limits (from Arduino)
    MIN_POSITION = 30
    MAX_POSITION = 160
    CENTER_POSITION = 90
    
    # Mapping from ROS standard angles to servo positions
    MIN_ANGLE = -45.0  # Maximum angle to port (negative)
    MAX_ANGLE = 45.0   # Maximum angle to starboard (positive)
    
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
        self.current_position = self.CENTER_POSITION
        self.target_position = self.CENTER_POSITION
        
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
            
            # Set initial position to center
            self.set_rudder_position(self.CENTER_POSITION)
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_rudder)
        
        self.get_logger().info('Rudder control node initialized')
    
    def angle_to_position(self, angle):
        """
        Convert from ROS standard angle (-45 to 45) to servo position (30 to 160)
        
        Args:
            angle: Angle in degrees (-45 to 45)
            
        Returns:
            Servo position value (30 to 160)
        """
        # Ensure angle is within limits
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle))
        
        # Map from angle range to position range
        normalized = (angle - self.MIN_ANGLE) / (self.MAX_ANGLE - self.MIN_ANGLE)
        position = int(self.MIN_POSITION + normalized * (self.MAX_POSITION - self.MIN_POSITION))
        
        return position
    
    def set_rudder_position(self, position):
        """
        Send position directly to Arduino via I2C
        
        Args:
            position: Servo position (30 to 160)
        """
        # Ensure position is within limits
        position = max(self.MIN_POSITION, min(self.MAX_POSITION, position))
        
        try:
            # Send position to Arduino
            self.bus.write_byte(self.rudder_address, position)
            self.current_position = position
            self.get_logger().debug(f"Sent rudder position: {position}")
            
            # Publish current position
            self.publish_position()
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending rudder position: {e}")
            return False
    
    def command_callback(self, msg):
        """
        Handle rudder command messages (angles)
        
        Args:
            msg: Float32 message with angle in degrees (-45 to 45)
        """
        angle = msg.data
        position = self.angle_to_position(angle)
        
        self.get_logger().info(f"Received angle command: {angle:.1f}° -> position: {position}")
        self.target_position = position
    
    def update_rudder(self):
        """Periodic update - send position to Arduino if needed"""
        if self.target_position != self.current_position:
            self.set_rudder_position(self.target_position)
    
    def publish_position(self):
        """Publish current rudder position"""
        msg = Float32()
        msg.data = float(self.current_position)
        self.position_publisher.publish(msg)
    
    def destroy_node(self):
        """Clean up and center rudder on shutdown"""
        try:
            # Return to center position
            self.set_rudder_position(self.CENTER_POSITION)
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