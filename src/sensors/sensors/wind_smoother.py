#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class WindSmootherNode(Node):
    """
    ROS2 Node for smoothing wind direction data
    
    Subscribes to raw wind direction data and publishes smoothed data.
    Currently just passes the data through, but will implement smoothing in the future.
    
    Subscribes to:
    - 'wind/raw_direction': Raw wind direction in degrees
    
    Publishes to:
    - 'wind/direction': Smoothed wind direction in degrees
    """
    
    def __init__(self):
        super().__init__('wind_smoother_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize state variables
        self.raw_wind_direction = 0.0
        self.smoothed_wind_direction = 0.0
        
        # Initialize publisher for smoothed data
        self.smoothed_publisher = self.create_publisher(
            Float32,
            'wind/direction',
            10
        )
        
        # Initialize subscriber for raw data
        self.raw_subscription = self.create_subscription(
            Float32,
            'wind/raw_direction',
            self.raw_wind_callback,
            10
        )
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_smoothing)
        
        self.get_logger().info('Wind smoother node initialized')
    
    def raw_wind_callback(self, msg: Float32):
        """Store latest raw wind direction data"""
        self.raw_wind_direction = msg.data
        self.get_logger().debug(f"Received raw wind direction: {self.raw_wind_direction:.1f}°")
    
    def update_smoothing(self):
        """
        Apply smoothing algorithm and publish smoothed wind direction.
        Currently just passes through the raw data.
        """
        # For now, just pass through the raw data
        # TODO: Implement actual smoothing logic in the future
        self.smoothed_wind_direction = self.raw_wind_direction
        
        # Publish smoothed direction
        msg = Float32()
        msg.data = float(self.smoothed_wind_direction)
        self.smoothed_publisher.publish(msg)
        
        self.get_logger().debug(f"Published smoothed wind direction: {self.smoothed_wind_direction:.1f}°")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WindSmootherNode()
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