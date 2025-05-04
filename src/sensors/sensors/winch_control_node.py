#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import struct
import time

class WinchControlNode(Node):
    """
    ROS2 Node for controlling the winch stepper motor via I2C
    
    Uses direct I2C communication with the winch Arduino (address 0x08)
    
    Subscribes to:
    - 'sail/control': Sail position command (0-100%)
    
    Publishes:
    - 'sail/position': Current position of the sail (0-100%)
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
        
        # Get parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.winch_address = self.get_parameter('winch_address').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_steps = self.get_parameter('max_steps').value
        
        # Initialize state variables
        self.current_position = 0.0  # 0-100%
        self.current_steps = 0
        
        # Initialize publishers
        self.position_publisher = self.create_publisher(
            Float32,
            'sail/position',
            10
        )
        
        # Initialize subscriber for sail control
        self.command_subscription = self.create_subscription(
            Float32,
            'sail/control',
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
    
    def position_to_steps(self, position_percent):
        """
        Convert from position percentage to number of steps
        
        Args:
            position_percent: Position as percentage (0-100%)
            
        Returns:
            Number of steps
        """
        return int((position_percent / 100.0) * self.max_steps)
    
    def command_callback(self, msg):
        """
        Handle sail control commands
        
        Args:
            msg: Float32 message with sail position as percentage (0-100%)
        """
        position = max(0.0, min(100.0, msg.data))
        
        self.get_logger().info(f"Received sail position command: {position:.1f}%")
        
        # Calculate target steps
        target_steps = self.position_to_steps(position)
        steps_diff = target_steps - self.current_steps
        
        if steps_diff != 0:
            # Set direction
            direction = 1 if steps_diff > 0 else 0
            self.set_direction(direction)
            
            # Send move command
            steps_to_move = abs(steps_diff)
            if self.move_steps(steps_to_move):
                self.current_steps = target_steps
                self.current_position = position
                self.get_logger().info(
                    f"Moving sail to {position:.1f}% "
                    f"({steps_to_move} steps, direction: {direction})"
                )
                
                # Publish updated position
                self.publish_position()
    
    def update_position(self):
        """Periodic update - publish current position"""
        self.publish_position()
    
    def publish_position(self):
        """Publish current sail position"""
        msg = Float32()
        msg.data = float(self.current_position)
        self.position_publisher.publish(msg)
    
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