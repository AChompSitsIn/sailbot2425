#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DummyWindNode(Node):
    def __init__(self):
        super().__init__('dummy_wind_node')
        
        # create publisher for wind direction
        self.publisher = self.create_publisher(
            Float32,
            'wind/direction',
            10
        )
        
        # create timer for publishing data every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Dummy Wind Node initialized')
        
    def timer_callback(self):
        msg = Float32()
        msg.data = 0.0  # wind direction in degrees
        
        self.publisher.publish(msg)
        self.get_logger().info('Published wind direction: 0.0 degrees')

def main(args=None):
    rclpy.init(args=args)
    node = DummyWindNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()