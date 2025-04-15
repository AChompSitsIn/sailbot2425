#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time
import threading

class RudderControlNode(Node):
    """
    ROS2 Node for controlling the rudder servo
    
    Subscribes to:
    - 'rudder/command': Angle in degrees to move the rudder
    
    Publishes:
    - 'rudder/position': Current position of the rudder
    - 'rudder/status': Status messages for logging/debugging
    """
    
    # Servo configuration
    SERVO_PIN = 33  # PWM5 on Jetson Orin Nano
    SERVO_MIN_DUTY = 2.5  # Duty cycle for minimum angle
    SERVO_MAX_DUTY = 12.5  # Duty cycle for maximum angle
    SERVO_FREQ = 50  # PWM frequency in Hz (20ms period)
    
    # Rudder limits and movement parameters
    MIN_ANGLE = -45.0  # Maximum angle to port (negative)
    MAX_ANGLE = 45.0   # Maximum angle to starboard (positive)
    CENTER_ANGLE = 0.0  # Center position
    MAX_ANGLE_CHANGE = 5.0  # Maximum change in degrees per command
    
    def __init__(self):
        super().__init__('rudder_control_node')
        
        # Declare and get parameters
        self.declare_parameter('servo_pin', self.SERVO_PIN)
        self.declare_parameter('min_angle', self.MIN_ANGLE)
        self.declare_parameter('max_angle', self.MAX_ANGLE)
        self.declare_parameter('max_angle_change', self.MAX_ANGLE_CHANGE)
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameter values
        self.servo_pin = self.get_parameter('servo_pin').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.max_angle_change = self.get_parameter('max_angle_change').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize state variables
        self.current_angle = self.CENTER_ANGLE
        self.target_angle = self.CENTER_ANGLE
        self.is_initialized = False
        self.lock = threading.Lock()
        
        # Initialize publishers
        self.position_publisher = self.create_publisher(
            Float32,
            'rudder/position',
            10
        )
        
        self.status_publisher = self.create_publisher(
            Float32,
            'rudder/status',
            10
        )
        
        # Initialize subscriber for rudder commands
        self.command_subscription = self.create_subscription(
            Float32,
            'rudder/command',
            self.command_callback,
            10
        )
        
        # Initialize GPIO and PWM
        self._initialize_hardware()
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_rudder)
        
        self.get_logger().info('Rudder control node initialized')
    
    def _initialize_hardware(self):
        """Initialize GPIO and PWM for servo control"""
        try:
            # Setup GPIO
            GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
            GPIO.setup(self.servo_pin, GPIO.OUT)
            
            # Setup PWM
            self.pwm = GPIO.PWM(self.servo_pin, self.SERVO_FREQ)
            
            # Start PWM with center position
            initial_duty = self._angle_to_duty_cycle(self.CENTER_ANGLE)
            self.pwm.start(initial_duty)
            
            self.is_initialized = True
            self.get_logger().info(
                f"Rudder servo initialized on pin {self.servo_pin} "
                f"at center position ({self.CENTER_ANGLE} degrees)"
            )
            
            # Publish initial position
            self._publish_position()
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize hardware: {e}")
            self.is_initialized = False
    
    def _angle_to_duty_cycle(self, angle):
        """
        Convert rudder angle to servo duty cycle
        
        Args:
            angle: Angle in degrees (negative = port, positive = starboard)
            
        Returns:
            Duty cycle for the PWM signal
        """
        # Map from rudder angle range to servo angle range (0-180)
        # We're assuming rudder angle range is from MIN_ANGLE to MAX_ANGLE
        # and needs to be mapped to 0-180 for the servo
        
        # First normalize angle to rudder range
        clamped_angle = max(self.min_angle, min(self.max_angle, angle))
        
        # Map from rudder range to servo range (0-180)
        servo_range = 180.0
        rudder_range = self.max_angle - self.min_angle
        
        # Calculate servo angle (0-180)
        servo_angle = ((clamped_angle - self.min_angle) / rudder_range) * servo_range
        
        # Convert servo angle to duty cycle
        return self.SERVO_MIN_DUTY + (servo_angle / 180.0) * (self.SERVO_MAX_DUTY - self.SERVO_MIN_DUTY)
    
    def command_callback(self, msg):
        """
        Handle rudder command messages
        
        Args:
            msg: Float32 message with angle in degrees
        """
        with self.lock:
            requested_angle = msg.data
            
            # Check if angle is within absolute limits
            if requested_angle < self.min_angle or requested_angle > self.max_angle:
                self.get_logger().warning(
                    f"Requested angle ({requested_angle}°) is outside "
                    f"allowed range ({self.min_angle}° to {self.max_angle}°). "
                    f"Clamping to limits."
                )
                requested_angle = max(self.min_angle, min(self.max_angle, requested_angle))
            
            # Check if angle change exceeds maximum change rate
            angle_change = abs(requested_angle - self.current_angle)
            if angle_change > self.max_angle_change:
                direction = 1 if requested_angle > self.current_angle else -1
                limited_angle = self.current_angle + (direction * self.max_angle_change)
                self.get_logger().warning(
                    f"Requested angle change ({angle_change}°) exceeds "
                    f"maximum allowed change ({self.max_angle_change}°). "
                    f"Limiting to {limited_angle}°."
                )
                self.target_angle = limited_angle
            else:
                self.target_angle = requested_angle
                
            self.get_logger().info(f"Rudder command received: {self.target_angle}°")
    
    def update_rudder(self):
        """Update rudder position based on target angle"""
        if not self.is_initialized:
            self.get_logger().warn_throttle(10, "Hardware not initialized, skipping rudder update")
            return
            
        with self.lock:
            if abs(self.target_angle - self.current_angle) > 0.1:
                # Update position
                self.current_angle = self.target_angle
                
                # Calculate duty cycle and update servo
                duty_cycle = self._angle_to_duty_cycle(self.current_angle)
                try:
                    self.pwm.ChangeDutyCycle(duty_cycle)
                    self.get_logger().debug(f"Updating rudder position to {self.current_angle}°")
                except Exception as e:
                    self.get_logger().error(f"Failed to update servo: {e}")
                
                # Publish updated position
                self._publish_position()
    
    def _publish_position(self):
        """Publish current rudder position"""
        msg = Float32()
        msg.data = float(self.current_angle)
        self.position_publisher.publish(msg)
    
    def destroy_node(self):
        """Clean up GPIO on shutdown"""
        self.get_logger().info("Shutting down rudder control")
        if hasattr(self, 'pwm') and self.is_initialized:
            try:
                # Return to center position before shutdown
                self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(self.CENTER_ANGLE))
                time.sleep(0.5)  # Give it time to center
                
                # Stop PWM and clean up GPIO
                self.pwm.stop()
                GPIO.cleanup(self.servo_pin)
                self.get_logger().info("Rudder servo shutdown complete")
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RudderControlNode()
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