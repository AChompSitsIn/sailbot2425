import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import math
import numpy as np
import json
from path_planning.path_planning.leg import Leg
from path_planning.path_planning.waypoint import Waypoint
from typing import List, Tuple, Optional


class PIDController:
    """Simple PID controller for rudder control"""
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = None

    def update(self, error, dt):
        """Update PID controller with new error value"""
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = 0
        if self.last_error is not None:
            D = self.Kd * (error - self.last_error) / dt
        self.last_error = error
        return P + I + D

    def reset(self):
        """Reset the PID controller state"""
        self.integral = 0
        self.last_error = None


class NavigationNode(Node):
    """
    Dedicated ROS2 node for autonomous navigation of the sailboat.

    This node:
    1. Handles path planning with tacking/jibing calculations
    2. Controls the rudder to follow planned paths
    3. Monitors progress and transitions between waypoints
    4. Publishes navigation status for monitoring
    5. Calculates optimal sail angle based on apparent wind
    """

    def __init__(self):
        super().__init__('navigation_node')

        # Navigation parameters
        self.declare_parameter('rudder_update_rate', 1.0)  # Hz
        self.declare_parameter('sail_update_rate', 1.0)    # Hz

        # PID parameters - these are from your friend's implementation
        self.declare_parameter('rudder_kp', 70.0)
        self.declare_parameter('rudder_ki', 0.5)
        self.declare_parameter('rudder_kd', 35.0)

        self.declare_parameter('waypoint_threshold', 5.0)   # meters

        # Get parameter values
        self.rudder_update_rate = self.get_parameter('rudder_update_rate').value
        self.sail_update_rate = self.get_parameter('sail_update_rate').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value

        # Get PID parameters
        kp = self.get_parameter('rudder_kp').value
        ki = self.get_parameter('rudder_ki').value
        kd = self.get_parameter('rudder_kd').value

        # Initialize PID controller
        self.rudder_pid = PIDController(Kp=kp, Ki=ki, Kd=kd)

        # Navigation state
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0      # in degrees, from GPS track or compass
        self.previous_lat = 0.0
        self.previous_lon = 0.0
        self.previous_heading = 0.0
        self.wind_direction = 0.0       # in degrees
        self.wind_speed = 5.0           # in m/s
        self.current_speed = 0.0        # in m/s
        self.rotational_velocity = 0.0  # in rad/s
        self.gps_timestamp = 0          # for calculating heading

        # Path planning
        self.leg_calculator = Leg()     # From path_planning
        self.active_course = []         # List of [lat, lon] waypoints
        self.current_target_idx = 0     # Index of current waypoint
        self.navigation_enabled = False # Whether we're in autonomous mode
        self.last_path_time = 0.0       # Time when path was last calculated

        # Path visualization data
        self.full_path = []             # Complete path including intermediates

        # Initialize path planning timer (slow rate - 1Hz)
        self.path_timer = self.create_timer(1.0, self.path_planning_callback)

        # Rudder control timer - 1Hz
        self.rudder_timer = self.create_timer(
            1.0/self.rudder_update_rate,  # 1.0 seconds
            self.rudder_control_callback
        )

        # Sail control timer - 1Hz
        self.sail_timer = self.create_timer(
            1.0/self.sail_update_rate,  # 1.0 seconds
            self.sail_control_callback
        )

        # Publishers
        self.rudder_publisher = self.create_publisher(
            Float32,
            'rudder/command',
            10
        )

        self.sail_angle_publisher = self.create_publisher(
            Float32,
            'sail/angle',  # New topic for sail angle
            10
        )

        self.nav_status_publisher = self.create_publisher(
            String,
            'navigation/status',
            10
        )

        # Subscribers
        # GPS position
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )

        # GPS speed
        self.speed_subscription = self.create_subscription(
            Float64,
            'gps/speed',
            self.speed_callback,
            10
        )

        # Wind direction
        self.wind_subscription = self.create_subscription(
            Float32,
            'wind/direction',
            self.wind_callback,
            10
        )

        # Navigation commands - enable/disable
        self.command_subscription = self.create_subscription(
            Int32,
            'navigation/command',
            self.command_callback,
            10
        )

        # Target waypoint - receive from event controller
        self.waypoint_subscription = self.create_subscription(
            String,
            'navigation/target',
            self.waypoint_callback,
            10
        )

        # Boat status subscription to monitor mode changes
        self.boat_status_subscription = self.create_subscription(
            String,
            'boat_status',
            self.boat_status_callback,
            10
        )

        self.get_logger().info("Navigation node initialized with PID controller")

    def gps_callback(self, msg: NavSatFix):
        """Handle GPS position updates and calculate heading directly from position changes"""
        timestamp = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds

        # Store previous position for heading calculation
        self.previous_lat = self.current_lat
        self.previous_lon = self.current_lon

        # Update current position
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Calculate heading from GPS track if we have previous position
        if self.previous_lat != 0.0 and self.previous_lon != 0.0:
            # Time elapsed since last GPS update
            dt = timestamp - self.gps_timestamp

            if dt > 0.1:  # Only update if enough time passed (100ms)
                # Calculate heading from position change
                dx = self.current_lon - self.previous_lon
                dy = self.current_lat - self.previous_lat

                if abs(dx) > 1e-7 or abs(dy) > 1e-7:  # Avoid tiny movements
                    # Convert to heading (0-360°, 0 = North, clockwise)
                    self.current_heading = math.degrees(math.atan2(dx, dy)) % 360

                    # Calculate rotational velocity (in rad/s)
                    # This will (might) be replaced with IMU data when available
                    heading_change = math.radians((self.current_heading - self.previous_heading + 180) % 360 - 180)
                    self.rotational_velocity = heading_change / dt
                    self.previous_heading = self.current_heading

        self.gps_timestamp = timestamp

        # If in autonomous mode, check if we've reached the target
        if self.navigation_enabled and self.active_course:
            self.check_waypoint_reached()

    def speed_callback(self, msg: Float64):
        """Handle GPS speed updates"""
        self.current_speed = msg.data

    def wind_callback(self, msg: Float32):
        """Handle wind direction updates"""
        self.wind_direction = msg.data

    def command_callback(self, msg: Int32):
        """Handle navigation commands"""
        command = msg.data

        if command == 1:  # Enable autonomous navigation
            self.navigation_enabled = True
            self.rudder_pid.reset()  # Reset PID controller when starting autonomous mode
            self.get_logger().info("Autonomous navigation enabled")
        elif command == 0:  # Disable autonomous navigation
            self.navigation_enabled = False
            self.get_logger().info("Autonomous navigation disabled")

    def boat_status_callback(self, msg: String):
        """Handle boat status updates to monitor mode changes"""
        try:
            status = json.loads(msg.data)
            mode = status.get('mode', '')
            
            # If mode is RC, disable autonomous navigation
            if mode == 'rc':
                if self.navigation_enabled:
                    self.navigation_enabled = False
                    self.get_logger().info("Navigation disabled - boat in RC mode")
            # If mode is autonomous, enable navigation 
            elif mode == 'autonomous':
                if not self.navigation_enabled:
                    self.navigation_enabled = True
                    self.rudder_pid.reset()
                    self.get_logger().info("Navigation enabled - boat in autonomous mode")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing boat status: {e}")

    def waypoint_callback(self, msg: String):
        """Handle new target waypoint from event controller"""
        try:
            # Parse waypoint from JSON
            waypoint_data = json.loads(msg.data)

            # Extract lat/lon from JSON
            lat = waypoint_data.get('lat', 0.0)
            lon = waypoint_data.get('lon', 0.0)

            self.get_logger().info(f"Received new target waypoint: lat={lat}, lon={lon}")

            # Calculate path to this waypoint
            self.calculate_path((self.current_lat, self.current_lon), (lat, lon))

            # Reset PID controller for new path
            self.rudder_pid.reset()

        except Exception as e:
            self.get_logger().error(f"Error processing waypoint: {e}")

    def calculate_path(self, start: Tuple[float, float], end: Tuple[float, float]):
        """Calculate path from start to end, considering wind direction"""
        self.get_logger().info(f"Calculating path from {start} to {end}")

        # Use Leg calculator to determine optimal path
        waypoints = self.leg_calculator.calculate_path(
            start,
            end,
            self.wind_direction,
            self.wind_speed,
            self.current_heading
        )

        # Update navigation state
        self.active_course = [start] + waypoints
        self.current_target_idx = 1  # First target is first waypoint after start
        self.last_path_time = self.get_clock().now().nanoseconds / 1e9

        # Log path details
        if len(waypoints) > 1:
            self.get_logger().info(
                f"Path requires tacking/jibing - {len(waypoints)} points"
            )
        else:
            self.get_logger().info("Direct path possible")

        # Store full path for visualization
        self.full_path = self.active_course

        # Publish navigation status update
        self.publish_status()

        return self.active_course

    def path_planning_callback(self):
        """
        Path planning timer callback - runs at slower rate
        Handles checking if we need to recalculate path
        """
        if not self.navigation_enabled:
            return

        # Check for significant wind direction changes
        # This would be a good spot to recalculate path if wind shifts

        # For now, just publish status updates
        self.publish_status()

    def calculate_apparent_wind_angle(self) -> float:
        """
        Calculate apparent wind angle relative to boat

        Returns:
            Apparent wind angle in degrees relative to boat heading
        """
        # Calculate boat velocity vector
        boat_vel_x = self.current_speed * math.sin(math.radians(self.current_heading))
        boat_vel_y = self.current_speed * math.cos(math.radians(self.current_heading))

        # Calculate true wind vector (assuming wind_direction is where wind is coming FROM)
        # Convert to where wind is going TO by adding 180°
        wind_to_direction = (self.wind_direction + 180) % 360
        true_wind_x = self.wind_speed * math.sin(math.radians(wind_to_direction))
        true_wind_y = self.wind_speed * math.cos(math.radians(wind_to_direction))

        # Calculate apparent wind (true wind - boat velocity)
        apparent_x = true_wind_x - boat_vel_x
        apparent_y = true_wind_y - boat_vel_y

        # Calculate apparent wind angle (global frame)
        apparent_angle_global = math.degrees(math.atan2(apparent_x, apparent_y)) % 360

        # Convert to angle relative to boat heading
        apparent_angle_relative = (apparent_angle_global - self.current_heading) % 360

        # Normalize to [-180, 180]
        if apparent_angle_relative > 180:
            apparent_angle_relative -= 360

        return apparent_angle_relative

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180] range"""
        angle = angle % 360
        if angle > 180:
            angle = -180 + (angle - 180)
        return angle

    def calculate_sail_angle(self) -> float:
        """
        Calculate optimal sail angle based on apparent wind
        Uses the angle of attack formula from the physics simulator

        Returns:
            Optimal sail angle in degrees
        """
        # Get apparent wind angle relative to boat
        apparent_wind = self.calculate_apparent_wind_angle()

        # Apply the angle of attack formula from the simulator
        # aoa(x) = (44/90) * x where x is normalized to [-180, 180]
        normalized_wind = self.normalize_angle(apparent_wind)
        sail_angle = (44.0 / 90.0) * normalized_wind

        # Clamp to reasonable sail angle range
        sail_angle = max(-90.0, min(90.0, sail_angle))

        return sail_angle

    def sail_control_callback(self):
        """
        Sail control timer callback - calculates and publishes optimal sail angle
        """
        if not self.navigation_enabled:
            return

        # Calculate optimal sail angle
        optimal_sail_angle = self.calculate_sail_angle()

        # Create and publish message
        msg = Float32()
        msg.data = float(optimal_sail_angle)
        self.sail_angle_publisher.publish(msg)

        # Log sail angle calculation
        self.get_logger().debug(
            f"Sail angle calculation: apparent_wind={self.calculate_apparent_wind_angle():.1f}°, "
            f"optimal_sail={optimal_sail_angle:.1f}°"
        )

    def rudder_control_callback(self):
        """
        Rudder control timer callback - runs at higher rate
        Updates rudder angle based on current position and heading
        """
        if not self.navigation_enabled:
            return
            
        if not self.active_course:
            return

        # Get current target waypoint
        target = self.get_current_target()
        if not target:
            return

        # Calculate angle to target
        target_heading = self.calculate_heading_to_target(target)

        # Calculate rudder angle using PID controller
        rudder_angle = self.calculate_rudder_angle_pid(target_heading)

        # Apply rudder angle
        self.set_rudder_angle(rudder_angle)

    def calculate_heading_to_target(self, target: Tuple[float, float]) -> float:
        """
        Calculate desired heading to target waypoint

        Args:
            target: Tuple of (lat, lon) for target position

        Returns:
            Desired heading in degrees (0-360)
        """
        # Calculate direction vector to target
        dx = target[1] - self.current_lon  # lon difference
        dy = target[0] - self.current_lat  # lat difference

        # Convert to heading (0-360°, 0 = North, clockwise)
        heading = math.degrees(math.atan2(dx, dy)) % 360

        return heading

    def print_angle(self, angle: float) -> float:
        """Normalize angle to be between -180 and 180 degrees"""
        angle = angle % 360
        if angle > 180:
            angle = -180 + (angle - 180)
        return angle

    def calculate_rudder_angle_pid(self, target_heading: float) -> float:
        """
        Calculate rudder angle using PID controller

        Args:
            target_heading: Desired heading in degrees

        Returns:
            Rudder angle command in degrees
        """
        # Calculate heading error
        heading_error = self.print_angle(target_heading - self.current_heading)

        # Scale error to [-1, 1] range for PID controller
        heading_error_scaled = heading_error / 180.0

        # Get time step (1.0 second for 1Hz update rate)
        dt = 1.0 / self.rudder_update_rate

        # Update PID controller
        rudder_correction = self.rudder_pid.update(heading_error_scaled, dt)

        # Apply limits and invert direction (matching friend's implementation)
        rudder_angle = max(min(rudder_correction, 20), -20) * -1

        # Debug logging
        self.get_logger().info(
            f"PID Rudder: target={target_heading:.1f}°, "
            f"current={self.current_heading:.1f}°, "
            f"error={heading_error:.1f}°, "
            f"scaled_error={heading_error_scaled:.3f}, "
            f"rudder={rudder_angle:.1f}°"
        )

        return rudder_angle

    def set_rudder_angle(self, angle: float):
        """Send rudder angle command"""
        # Clamp angle to reasonable range
        angle = max(-45.0, min(45.0, angle))

        # Create and publish message
        msg = Float32()
        msg.data = float(angle)
        self.rudder_publisher.publish(msg)

    def get_current_target(self) -> Optional[Tuple[float, float]]:
        """Get current target waypoint from active course"""
        if (not self.active_course or
            self.current_target_idx >= len(self.active_course)):
            return None

        return self.active_course[self.current_target_idx]

    def check_waypoint_reached(self):
        """Check if current target waypoint has been reached"""
        target = self.get_current_target()
        if not target:
            return False

        # Calculate distance to target using haversine formula
        distance = self.calculate_distance(
            self.current_lat, self.current_lon,
            target[0], target[1]
        )

        # Check if target reached
        if distance < self.waypoint_threshold:
            self.get_logger().info(
                f"Waypoint {self.current_target_idx} reached: {target} "
                f"(distance: {distance:.1f}m)"
            )

            # Move to next waypoint
            self.current_target_idx += 1

            # Check if we've completed the course
            if self.current_target_idx >= len(self.active_course):
                self.get_logger().info("Navigation course completed")
            else:
                next_target = self.get_current_target()
                self.get_logger().info(f"New target: {next_target}")

            # Publish updated status
            self.publish_status()

            return True

        return False

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points in meters"""
        # Haversine formula
        R = 6371000  # Earth radius in meters

        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = (math.sin(dlat/2)**2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def publish_status(self):
        """Publish navigation status"""
        # Prepare status data
        status = {
            "enabled": self.navigation_enabled,
            "current_position": [self.current_lat, self.current_lon],
            "heading": self.current_heading,
            "speed": self.current_speed,
            "wind_direction": self.wind_direction,
            "apparent_wind_angle": self.calculate_apparent_wind_angle(),
            "optimal_sail_angle": self.calculate_sail_angle(),
            "active_course": self.active_course,
            "current_target_idx": self.current_target_idx,
            "full_path": self.full_path,
            "pid_gains": {
                "kp": self.rudder_pid.Kp,
                "ki": self.rudder_pid.Ki,
                "kd": self.rudder_pid.Kd
            }
        }

        # Add current target if available
        target = self.get_current_target()
        if target:
            status["current_target"] = target

            # Calculate distance to target
            distance = self.calculate_distance(
                self.current_lat, self.current_lon,
                target[0], target[1]
            )
            status["distance_to_target"] = distance

        # Convert to JSON
        status_json = json.dumps(status)

        # Publish status
        msg = String()
        msg.data = status_json
        self.nav_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
