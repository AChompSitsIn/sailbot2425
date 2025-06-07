#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class WindSensorNode(Node):
    """
    ROS2 Node for reading wind sensor data from the Arduino
    
    Uses USB serial communication with the Arduino for wind sensor data
    
    Publishes:
    - 'wind/direction': Wind direction in degrees
    """
    
    # Serial configuration
    SERIAL_PORT = '/dev/arduino_wind'
    BAUD_RATE = 115200
    
    def __init__(self):
        super().__init__('wind_sensor_node')
        
        # Declare and get parameters
        self.declare_parameter('serial_port', self.SERIAL_PORT)
        self.declare_parameter('baud_rate', self.BAUD_RATE)
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize state variables
        self.wind_direction = 0.0
        self.ser = None
        
        # Initialize publishers
        self.wind_publisher = self.create_publisher(
            Float32,
            'wind/raw_direction',
            10
        )
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(
                f"Serial connection established on {self.serial_port} at {self.baud_rate} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize serial connection: {e}")
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_wind_data)
        
        self.get_logger().info('Wind sensor node initialized')
    
    def read_wind_data(self):
        """Read wind direction from the Arduino via serial"""
        if not self.ser or not self.ser.is_open:
            return None
            
        try:
            # Read lines until we get wind data or timeout
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Check if it's wind data
                if line.startswith("WIND,"):
                    try:
                        _, degrees = line.split(',')
                        wind_degrees = float(degrees)
                        return wind_degrees
                    except ValueError:
                        self.get_logger().error(f"Error parsing wind data: {line}")
                        return None
                
                # Check if it's an error
                elif line.startswith("ERROR,"):
                    self.get_logger().error(f"Sensor error: {line}")
                    return None
                    
            return None
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
            
            self.get_logger().debug(f"Published raw wind direction: {self.wind_direction:.1f}°")
    
    def destroy_node(self):
        """Clean up serial connection on shutdown"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("Serial port closed")
        except Exception as e:
            self.get_logger().error(f"Error closing serial port: {e}")
        
        super().destroy_node()

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
