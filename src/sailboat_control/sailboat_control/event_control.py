from abc import ABC, abstractmethod
from .common import ControlMode
from path_planning.path_planning.waypoint import Waypoint
from path_planning.path_planning.leg import Leg
from typing import List, Optional, Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float64, String
from .buoy_detection import BuoyDetector
from std_msgs.msg import String, Int32
import json

class EventControl(ABC):
    """base class for event control"""
    def __init__(self, waypoints: List[Waypoint], node: Node):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.node = node  # ros2 node for subscriptions
        
        # latest sensor data storage
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_speed = 0.0
        self.wind_direction = 0.0
        self.gps_time = ""
        self.satellites = 0
        
        # set up buoy detector with default 5 meter threshold
        self.buoy_detector = BuoyDetector(threshold_distance=5.0)
        
        # set up subscriptions
        self._setup_subscriptions()

        # publishers for navigation system
        self.waypoint_publisher = node.create_publisher(
            String,
            'navigation/target',
            10
        )
        
        self.nav_command_publisher = node.create_publisher(
            Int32,
            'navigation/command',
            10
        )
    
    def _setup_subscriptions(self):
        """setup gps and wind sensor subscriptions"""
        self.gps_subscription = self.node.create_subscription(
            NavSatFix,
            'gps/fix',
            self._gps_callback,
            10
        )
        
        # New subscription for GPS speed
        self.speed_subscription = self.node.create_subscription(
            Float64,
            'gps/speed',
            self._speed_callback,
            10
        )
        
        # New subscription for GPS time
        self.time_subscription = self.node.create_subscription(
            String,
            'gps/time',
            self._time_callback,
            10
        )
        
        self.wind_subscription = self.node.create_subscription(
            Float32,
            'wind/direction',
            self._wind_callback,
            10
        )
    
    def _gps_callback(self, msg: NavSatFix):
        """store latest gps position data and check for proximity to buoys"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Check if we're near any buoys
        newly_reached_buoys = self.buoy_detector.check_buoy_proximity(
            (self.current_lat, self.current_lon), 
            self.waypoints
        )
        
        # Log any newly reached buoys
        for buoy in newly_reached_buoys:
            self.node.get_logger().info(
                f"Reached buoy at lat={buoy.lat}, lon={buoy.long} "
                f"(pass #{buoy.n_passed})"
            )
            
            # Update the current waypoint index based on the most recently passed buoy
            current_buoy = self.buoy_detector.get_current_buoy()
            if current_buoy is not None:
                # Find the index of the current buoy
                try:
                    current_index = self.waypoints.index(current_buoy)
                    # Set the next waypoint index
                    self.current_waypoint_index = (current_index + 1) % len(self.waypoints)
                    
                    next_waypoint = self.waypoints[self.current_waypoint_index]
                    self.node.get_logger().info(
                        f"Next target: Waypoint at lat={next_waypoint.lat}, lon={next_waypoint.long}"
                    )
                except ValueError:
                    # Buoy not in waypoints list (should not happen)
                    self.node.get_logger().error("Current buoy not found in waypoints list!")
        
        # Log position updates for debugging
        self.node.get_logger().debug(f"Position update: lat={self.current_lat:.6f}, lon={self.current_lon:.6f}")
    
    def _speed_callback(self, msg: Float64):
        """store latest speed data"""
        self.current_speed = msg.data
        self.node.get_logger().debug(f"Speed update: {self.current_speed:.2f} m/s")
    
    def _time_callback(self, msg: String):
        """store latest gps time data"""
        self.gps_time = msg.data
        self.node.get_logger().debug(f"GPS time update: {self.gps_time}")
    
    def _wind_callback(self, msg: Float32):
        """store latest wind direction"""
        self.wind_direction = msg.data
        self.node.get_logger().debug(f"Wind direction update: {self.wind_direction:.1f}°")
    
    def get_current_position(self) -> Tuple[float, float]:
        """get latest position"""
        return (self.current_lat, self.current_lon)
    
    def get_current_speed(self) -> float:
        """get latest speed in m/s"""
        return self.current_speed
    
    def get_gps_time(self) -> str:
        """get latest gps time"""
        return self.gps_time
    
    def get_wind_direction(self) -> float:
        """get latest wind direction"""
        return self.wind_direction
    
    def get_current_buoy(self) -> Optional[Waypoint]:
        """get the most recently passed buoy"""
        return self.buoy_detector.get_current_buoy()
    
    def get_next_waypoint(self) -> Optional[Waypoint]:
        """get the next target waypoint based on most recently passed buoy"""
        if not self.waypoints:
            return None
            
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        
        return None
    
    def set_buoy_threshold(self, threshold_meters: float):
        """set the threshold distance for buoy detection"""
        self.buoy_detector.threshold_distance = threshold_meters
        self.node.get_logger().info(f"Buoy detection threshold set to {threshold_meters} meters")
    
    def reset_buoy_detection(self):
        """reset buoy detection state, marking all buoys as not reached"""
        self.buoy_detector.reset()
        for buoy in self.waypoints:
            buoy.marked = False
            buoy.n_passed = 0
        self.current_waypoint_index = 0
        self.node.get_logger().info("Buoy detection state reset")
    
    def get_reached_buoys_count(self) -> int:
        """get the number of buoys that have been reached at least once"""
        return self.buoy_detector.get_reached_buoys_count()

    def handle_rc(self):
        """standard rc control implementation for all events"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        speed = self.get_current_speed()
        # implement standard rc control logic here
        pass

    def enable_autonomous_navigation(self):
    """Enable autonomous navigation"""
    msg = Int32()
    msg.data = 1  # 1 = enable
    self.nav_command_publisher.publish(msg)
    self.node.get_logger().info("Enabled autonomous navigation")
    
    def disable_autonomous_navigation(self):
        """Disable autonomous navigation"""
        msg = Int32()
        msg.data = 0  # 0 = disable
        self.nav_command_publisher.publish(msg)
        self.node.get_logger().info("Disabled autonomous navigation")
        
    def send_waypoint(self, waypoint: Waypoint):
        """Send waypoint to navigation system"""
        # Create JSON waypoint data
        waypoint_data = {
            'lat': waypoint.lat,
            'lon': waypoint.long
        }
        
        # Convert to JSON string
        waypoint_json = json.dumps(waypoint_data)
        
        # Create and publish message
        msg = String()
        msg.data = waypoint_json
        self.waypoint_publisher.publish(msg)
        
        self.node.get_logger().info(
            f"Sent waypoint to navigation: lat={waypoint.lat}, lon={waypoint.long}"
        )


class FleetRaceControl(EventControl):
    """pure rc control - no autonomous capability"""
    pass  # only uses the base rc control

class PrecisionNavigationControl(EventControl):
    def handle_autonomous(self):
        """precision navigation autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        speed = self.get_current_speed()
        
        # get next waypoint based on most recently passed buoy
        next_waypoint = self.get_next_waypoint()
        if next_waypoint:
            # implement precision navigation autonomous behavior
            pass

class StationKeepingControl(EventControl):
    def handle_autonomous(self):
        """station keeping autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        speed = self.get_current_speed()
        
        # get station keeping target point
        next_waypoint = self.get_next_waypoint()
        if next_waypoint:
            # implement station keeping autonomous behavior
            pass

class EnduranceControl(EventControl):
    def handle_autonomous(self):
        """endurance autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        speed = self.get_current_speed()
        
        # get next waypoint based on most recently passed buoy
        next_waypoint = self.get_next_waypoint()
        if next_waypoint:
            # implement endurance autonomous behavior
            pass

class PayloadControl(EventControl):
    def handle_autonomous(self):
        """payload delivery autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        speed = self.get_current_speed()
        
        # get next waypoint based on most recently passed buoy
        next_waypoint = self.get_next_waypoint()
        if next_waypoint:
            # implement payload delivery autonomous behavior
            pass

class DeveloperControl(EventControl):
    def handle_autonomous(self):
        """developer testing autonomous control"""
        # Get next waypoint
        next_waypoint = self.get_next_waypoint()
        
        if next_waypoint:
            # Enable autonomous navigation
            self.enable_autonomous_navigation()
            
            # Send waypoint to navigation system
            self.send_waypoint(next_waypoint)
        else:
            self.node.get_logger().warning("No waypoint available for navigation")

def create_event_control(event_type: str, waypoints: List[Waypoint], node: Node) -> EventControl:
    """factory function to create appropriate event control"""
    control_classes = {
        "fleet_race": FleetRaceControl,
        "precision_navigation": PrecisionNavigationControl,
        "station_keeping": StationKeepingControl,
        "endurance": EnduranceControl,
        "payload": PayloadControl,
        "developer_mode": DeveloperControl
    }
    
    control_class = control_classes.get(event_type.lower())
    if control_class:
        return control_class(waypoints, node)
    else:
        raise ValueError(f"Unknown event type: {event_type}")