#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import math
import time

class TestControlNode(Node):
    """
    ROS2 Node for testing rudder and sail control systems
    
    Publishes to:
    - 'rudder/command': Angle commands for rudder (-45 to 45 degrees)
    - 'sail/control': Position commands for sail (0-100%)
    """
    
    def __init__(self):
        super().__init__('test_control_node')
        
        # Declare parameters
        self.declare_parameter('test_mode', 'oscillation')  # options: oscillation, step, sweep
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('run_time', 60.0)  # seconds
        
        # Get parameters
        self.test_mode = self.get_parameter('test_mode').value
        self.update_rate = self.get_parameter('update_rate').value
        self.run_time = self.get_parameter('run_time').value
        
        # Initialize publishers
        self.rudder_publisher = self.create_publisher(
            Float32,
            'rudder/command',
            10
        )
        
        self.sail_publisher = self.create_publisher(
            Float32,
            'sail/control',
            10
        )
        
        # Initialize test variables
        self.start_time = time.time()
        self.counter = 0
        
        # Create timer for test sequence
        self.timer = self.create_timer(1.0/self.update_rate, self.test_callback)
        
        self.get_logger().info(f'Test control node initialized with mode: {self.test_mode}')
        self.get_logger().info(f'Will run for {self.run_time} seconds at {self.update_rate} Hz')
    
    def publish_rudder_command(self, angle):
        """
        Publish a rudder command
        
        Args:
            angle: Rudder angle (-45 to 45 degrees)
        """
        # Ensure angle is within limits
        angle = max(-45.0, min(45.0, angle))
        
        msg = Float32()
        msg.data = float(angle)
        self.rudder_publisher.publish(msg)
        
        self.get_logger().info(f'Published rudder command: {angle:.1f}°')
    
    def publish_sail_command(self, position):
        """
        Publish a sail position command
        
        Args:
            position: Sail position (0-100%)
        """
        # Ensure position is within limits
        position = max(0.0, min(100.0, position))
        
        msg = Float32()
        msg.data = float(position)
        self.sail_publisher.publish(msg)
        
        self.get_logger().info(f'Published sail command: {position:.1f}%')
    
    def test_callback(self):
        """Execute test sequence based on selected mode"""
        elapsed_time = time.time() - self.start_time
        
        # Check if we've reached the run time limit
        if elapsed_time > self.run_time:
            # Return to neutral position before stopping
            self.publish_rudder_command(0.0)
            self.publish_sail_command(0.0)
            self.get_logger().info('Test sequence completed')
            
            # Stop the timer
            self.timer.cancel()
            return
        
        # Execute the selected test pattern
        if self.test_mode == 'oscillation':
            self.run_oscillation_test(elapsed_time)
        elif self.test_mode == 'step':
            self.run_step_test()
        elif self.test_mode == 'sweep':
            self.run_sweep_test(elapsed_time)
        else:
            self.get_logger().error(f'Unknown test mode: {self.test_mode}')
            self.timer.cancel()
    
    def run_oscillation_test(self, elapsed_time):
        """
        Run oscillation test - sine wave pattern
        
        Args:
            elapsed_time: Elapsed time in seconds
        """
        # Calculate sine wave for rudder (-45 to 45 degrees)
        # Period of 10 seconds
        rudder_angle = 45.0 * math.sin(2 * math.pi * elapsed_time / 10.0)
        
        # Calculate sine wave for sail (20 to 80%)
        # Period of 20 seconds, offset from rudder
        sail_position = 50.0 + 30.0 * math.sin(2 * math.pi * elapsed_time / 20.0)
        
        # Publish commands
        self.publish_rudder_command(rudder_angle)
        self.publish_sail_command(sail_position)
    
    def run_step_test(self):
        """Run step test - alternate between positions"""
        self.counter += 1
        step = self.counter % 4
        
        if step == 0:
            self.publish_rudder_command(0.0)
            self.publish_sail_command(0.0)
        elif step == 1:
            self.publish_rudder_command(30.0)
            self.publish_sail_command(25.0)
        elif step == 2:
            self.publish_rudder_command(0.0)
            self.publish_sail_command(50.0)
        elif step == 3:
            self.publish_rudder_command(-30.0)
            self.publish_sail_command(75.0)
    
    def run_sweep_test(self, elapsed_time):
        """
        Run sweep test - slowly sweep through full range
        
        Args:
            elapsed_time: Elapsed time in seconds
        """
        # Sweep period of 30 seconds
        cycle_position = (elapsed_time % 30.0) / 30.0
        
        # Triangle wave for rudder (-45 to 45 degrees)
        if cycle_position < 0.5:
            # Ramp up from -45 to 45
            normalized = cycle_position * 2.0
            rudder_angle = -45.0 + normalized * 90.0
        else:
            # Ramp down from 45 to -45
            normalized = (cycle_position - 0.5) * 2.0
            rudder_angle = 45.0 - normalized * 90.0
        
        # Sawtooth wave for sail (0 to 100%)
        sail_position = (elapsed_time % 15.0) / 15.0 * 100.0
        
        # Publish commands
        self.publish_rudder_command(rudder_angle)
        self.publish_sail_command(sail_position)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        # Return to neutral position
        try:
            self.publish_rudder_command(0.0)
            self.publish_sail_command(0.0)
            self.get_logger().info("Returned to neutral position on shutdown")
        except Exception as e:
            self.get_logger().error(f"Error on shutdown: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestControlNode()
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