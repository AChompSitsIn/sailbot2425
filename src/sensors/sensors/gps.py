#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class DummyGPSNode(Node):
    def __init__(self):
        super().__init__('dummy_gps_node')
        
        # create publisher for gps data
        self.publisher = self.create_publisher(
            NavSatFix,
            'gps/fix',
            10
        )
        
        # create timer for publishing data every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Dummy GPS Node initialized')
        
    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = 0.0
        msg.longitude = 0.0
        msg.altitude = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info('Published GPS data: lat=0.0, lon=0.0')

def main(args=None):
    rclpy.init(args=args)
    node = DummyGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()