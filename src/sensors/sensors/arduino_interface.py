#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import time
import struct
import threading

class ArduinoInterfaceNode(Node):
    """
    ROS2 Node for interfacing with Arduino via I2C.
    
    Receives wind direction and sail position data from Arduino.
    Sends sail control commands to Arduino
    """
    
    # I2C configuration
    DEFAULT_ARDUINO_ADDRESS = 0x08
    DEFAULT_I2C_BUS = 7  # Arduino detected on bus 7
    
    # Arduino command IDs
    CMD_CHANGE_POLLING_RATE = 0x01
    CMD_REQUEST_REGISTER = 0x02
    CMD_SAIL_CONTROL = 0x03
    
    def __init__(self):
        super().__init__('arduino_interface_node')
        
        # Declare and get parameters
        self.declare_parameter('i2c_bus', self.DEFAULT_I2C_BUS)
        self.declare_parameter('arduino_address', self.DEFAULT_ARDUINO_ADDRESS)
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.arduino_address = self.get_parameter('arduino_address').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize publishers
        self.wind_publisher = self.create_publisher(
            Float32,
            'wind/direction',
            10
        )
        
        self.sail_position_publisher = self.create_publisher(
            Float32,
            'sail/position',
            10
        )
        
        # Initialize subscriber for sail control
        self.sail_control_subscription = self.create_subscription(
            Float32,
            'sail/control',
            self.sail_control_callback,
            10
        )
        
        # Initialize I2C communication
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            self.get_logger().info(
                f"I2C initialized on bus {self.i2c_bus}, "
                f"device address 0x{self.arduino_address:02X}"
            )
        except Exception as e:
            self.get_logger().error(f"Error initializing I2C: {e}")
            raise
        
        # Data storage
        self.wind_direction = 0.0
        self.sail_position = 0.0
        
        # Create timer for periodic reading
        self.timer = self.create_timer(1.0/self.update_rate, self.timer_callback)
        
        # Thread lock for I2C access
        self.i2c_lock = threading.Lock()
        
        self.get_logger().info("Arduino interface node initialized")
    
    def read_arduino_data(self):
        """Read wind direction and sail position from Arduino over I2C."""
        try:
            with self.i2c_lock:
                # Use the block reading method to get both wind direction and sail position (8 bytes)
                block_data = self.bus.read_i2c_block_data(self.arduino_address, 0, 8)
                
                # Convert the first 4 bytes to wind direction float
                wind_degrees = struct.unpack('f', bytes(block_data[0:4]))[0]
                
                # Convert the next 4 bytes to sail position float
                sail_position = struct.unpack('f', bytes(block_data[4:8]))[0]
                
                return wind_degrees, sail_position
        except Exception as e:
            self.get_logger().error(f"Error reading data from Arduino: {e}")
            return None, None
    
    def send_command(self, command_id, params=None):
        """Send a command to the Arduino."""
        try:
            with self.i2c_lock:
                # Prepare command packet
                command_packet = [command_id]
                if params:
                    if isinstance(params, list):
                        command_packet.extend(params)
                    else:
                        command_packet.append(params)
                
                # Send command to Arduino
                self.bus.write_i2c_block_data(
                    self.arduino_address, 
                    command_packet[0], 
                    command_packet[1:] if len(command_packet) > 1 else []
                )
                return True
        except Exception as e:
            self.get_logger().error(f"Error sending command to Arduino: {e}")
            return False
    
    def timer_callback(self):
        """Read data from Arduino and publish to ROS topics."""
        # Read wind direction and sail position
        wind_direction, sail_position = self.read_arduino_data()
        
        if wind_direction is not None:
            # Update stored value
            self.wind_direction = wind_direction
            
            # Create and publish wind direction message
            wind_msg = Float32()
            wind_msg.data = float(wind_direction)
            self.wind_publisher.publish(wind_msg)
            
            self.get_logger().debug(f"Published wind direction: {wind_direction:.1f}°")
        
        if sail_position is not None:
            # Update stored value
            self.sail_position = sail_position
            
            # Create and publish sail position message
            sail_msg = Float32()
            sail_msg.data = float(sail_position)
            self.sail_position_publisher.publish(sail_msg)
            
            self.get_logger().debug(f"Published sail position: {sail_position:.1f}")
    
    def sail_control_callback(self, msg):
        """Handle sail control commands."""
        sail_position = int(msg.data)
        
        # Ensure value is in valid range (0-255)
        sail_position = max(0, min(255, sail_position))
        
        # Send sail control command to Arduino
        if self.send_command(self.CMD_SAIL_CONTROL, sail_position):
            self.get_logger().info(f"Sent sail control command: {sail_position}")
        else:
            self.get_logger().error(f"Failed to send sail control command: {sail_position}")
    
    def change_polling_rate(self, rate_in_deciseconds):
        """Change the polling rate of the wind sensor."""
        return self.send_command(self.CMD_CHANGE_POLLING_RATE, rate_in_deciseconds)
    
    def request_specific_register(self, register_address):
        """Request a specific Modbus register from the wind sensor."""
        return self.send_command(self.CMD_REQUEST_REGISTER, register_address)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduinoInterfaceNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in Arduino interface node: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()