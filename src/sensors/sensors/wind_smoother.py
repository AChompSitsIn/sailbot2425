#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque
import numpy as np
import math

class WindSmootherNode(Node):
    """
    ROS2 Node for smoothing wind direction data using a sliding window median filter.
    
    Uses a circular median calculation to properly handle the 0-360 degree wrap-around.
    
    Subscribes to:
    - 'wind/raw_direction': Raw wind direction in degrees
    
    Publishes to:
    - 'wind/direction': Smoothed wind direction in degrees
    """
    
    def __init__(self):
        super().__init__('wind_smoother_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('window_size', 20)   # Number of samples for median
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.window_size = self.get_parameter('window_size').value
        
        # Initialize state variables
        self.wind_buffer = deque(maxlen=self.window_size)  # Sliding window buffer
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
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_smoothed_data)
        
        self.get_logger().info(f'Wind smoother node initialized with window size: {self.window_size}')
    
    def raw_wind_callback(self, msg: Float32):
        """Store latest raw wind direction data in the sliding window buffer"""
        raw_direction = msg.data
        
        # Add to sliding window buffer (automatically removes oldest if full)
        self.wind_buffer.append(raw_direction)
        
        # Calculate smoothed value
        self.smoothed_wind_direction = self.calculate_circular_median()
        
        self.get_logger().debug(
            f"Raw: {raw_direction:.1f}°, Buffer size: {len(self.wind_buffer)}, "
            f"Smoothed: {self.smoothed_wind_direction:.1f}°"
        )
    
    def calculate_circular_median(self):
        """
        Calculate the median of circular data (wind directions).
        
        This handles the wrap-around at 0/360 degrees by converting to unit vectors,
        averaging them, and converting back to an angle.
        
        Returns:
            Median wind direction in degrees (0-360)
        """
        if not self.wind_buffer:
            return 0.0
        
        # For a single value, just return it
        if len(self.wind_buffer) == 1:
            return self.wind_buffer[0]
        
        # Convert degrees to radians for calculations
        angles_rad = [math.radians(angle) for angle in self.wind_buffer]
        
        # Method 1: Vector averaging for circular median
        # Convert to unit vectors
        x_components = [math.cos(angle) for angle in angles_rad]
        y_components = [math.sin(angle) for angle in angles_rad]
        
        # Calculate mean vector
        mean_x = np.mean(x_components)
        mean_y = np.mean(y_components)
        
        # Convert back to angle
        mean_angle_rad = math.atan2(mean_y, mean_x)
        mean_angle_deg = math.degrees(mean_angle_rad)
        
        # Normalize to 0-360 range
        if mean_angle_deg < 0:
            mean_angle_deg += 360
        
        # Alternative Method: Find the angle that minimizes circular distance
        # This is more computationally intensive but can be more robust
        if len(self.wind_buffer) >= 5:  # Only use for sufficient data
            median_angle = self.find_circular_median_minimizing_distance()
            return median_angle
        
        return mean_angle_deg
    
    def find_circular_median_minimizing_distance(self):
        """
        Find the angle that minimizes the sum of circular distances to all other angles.
        This is a more robust circular median calculation.
        
        Returns:
            Circular median in degrees (0-360)
        """
        min_total_distance = float('inf')
        median_angle = 0.0
        
        # Check each angle in the buffer as a potential median
        for candidate in self.wind_buffer:
            total_distance = 0.0
            
            # Calculate sum of circular distances from this candidate to all others
            for angle in self.wind_buffer:
                # Circular distance between two angles
                diff = abs(candidate - angle)
                if diff > 180:
                    diff = 360 - diff
                total_distance += diff
            
            # Update if this candidate has lower total distance
            if total_distance < min_total_distance:
                min_total_distance = total_distance
                median_angle = candidate
        
        return median_angle
    
    def circular_distance(self, angle1, angle2):
        """
        Calculate the shortest angular distance between two angles.
        
        Args:
            angle1, angle2: Angles in degrees
            
        Returns:
            Shortest angular distance in degrees (0-180)
        """
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = 360 - diff
        return diff
    
    def publish_smoothed_data(self):
        """
        Publish the smoothed wind direction.
        Only publishes if we have data in the buffer.
        """
        if self.wind_buffer:
            # Create and publish message
            msg = Float32()
            msg.data = float(self.smoothed_wind_direction)
            self.smoothed_publisher.publish(msg)
            
            # Log statistics periodically (every 10th update)
            if hasattr(self, '_update_count'):
                self._update_count += 1
            else:
                self._update_count = 0
                
            if self._update_count % 10 == 0:
                # Calculate some statistics for logging
                if len(self.wind_buffer) > 1:
                    # Calculate circular standard deviation
                    angles_rad = [math.radians(angle) for angle in self.wind_buffer]
                    x_components = [math.cos(angle) for angle in angles_rad]
                    y_components = [math.sin(angle) for angle in angles_rad]
                    
                    r = math.sqrt(np.mean(x_components)**2 + np.mean(y_components)**2)
                    circular_std = math.degrees(math.sqrt(-2 * math.log(r))) if r > 0 else 0
                    
                    self.get_logger().info(
                        f"Wind smoother stats - Buffer: {len(self.wind_buffer)}/{self.window_size}, "
                        f"Smoothed: {self.smoothed_wind_direction:.1f}°, "
                        f"Circular StdDev: {circular_std:.1f}°"
                    )

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