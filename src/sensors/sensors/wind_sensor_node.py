#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import struct
import time

class WindSensorNode(Node):
    """
    ROS2 Node for reading wind sensor data from the Arduino
    
    Uses I2C communication with the Arduino for wind sensor data
    
    Publishes:
    - 'wind/direction': Wind direction in degrees
    """
    
    # Arduino I2C configuration (using the winch Arduino)
    ARDUINO_ADDRESS = 0x08
    I2C_BUS = 1  # Use I2C bus 1
    
    def __init__(self):
        super().__init__('wind_sensor_node')
        
        # Declare and get parameters
        self.declare_parameter('i2c_bus', self.I2C_BUS)
        self.declare_parameter('arduino_address', self.ARDUINO_ADDRESS)
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.arduino_address = self.get_parameter('arduino_address').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize state variables
        self.wind_direction = 0.0
        
        # Initialize publishers
        self.wind_publisher = self.create_publisher(
            Float32,
            'wind/direction',
            10
        )
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            self.get_logger().info(
                f"I2C initialized on bus {self.i2c_bus}, "
                f"address 0x{self.arduino_address:02X}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_wind_data)
        
        self.get_logger().info('Wind sensor node initialized')
    
    def read_wind_data(self):
        """Read wind direction from the Arduino"""
        try:
            # Read 4 bytes of wind direction data (float)
            block_data = self.bus.read_i2c_block_data(self.arduino_address, 0, 8)
            
            # Extract wind direction from first 4 bytes
            wind_degrees = struct.unpack('f', bytes(block_data[0:4]))[0]
            
            return wind_degrees
        except Exception as e:
            self.get_logger().error(f"Error reading wind data: {e}")
            return None
    
    def update_wind_data(self):
        """Periodic update - read and publish wind data"""
        wind_direction = self.read_wind_data()
        
        if wind_direction is not None:
            self.wind_direction = wind_direction
            
            # Publish wind direction
            msg = Float32()
            msg.data = float(self.wind_direction)
            self.wind_publisher.publish(msg)
            
            self.get_logger().debug(f"Published wind direction: {self.wind_direction:.1f}°")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WindSensorNode()
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