#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import pkgutil

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        
        # Print Python path
        self.get_logger().info("Python path:")
        for path in sys.path:
            self.get_logger().info(f"  {path}")
        
        # Try to find path_planning
        self.get_logger().info("\nAvailable modules:")
        for finder, name, ispkg in pkgutil.iter_modules():
            if "path" in name:
                self.get_logger().info(f"  {name} ({finder.path})")
        
        # Try direct import
        try:
            import path_planning
            self.get_logger().info(f"\npath_planning found at: {path_planning.__file__}")
            self.get_logger().info(f"path_planning contains: {dir(path_planning)}")
        except ImportError as e:
            self.get_logger().info(f"\nFailed to import path_planning: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()