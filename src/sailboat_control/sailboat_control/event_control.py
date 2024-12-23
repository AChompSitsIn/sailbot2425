from abc import ABC, abstractmethod
from .boat import ControlMode
from path_planning.path_planning.waypoint import Waypoint
from path_planning.path_planning.leg import leg
from typing import List, Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

class EventControl(ABC):
    """base class for event control"""
    def __init__(self, waypoints: List[Waypoint], node: Node):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.node = node  # ros2 node for subscriptions
        
        # latest sensor data storage
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.wind_direction = 0.0
        
        # set up subscriptions
        self._setup_subscriptions()
    
    def _setup_subscriptions(self):
        """setup gps and wind sensor subscriptions"""
        self.gps_subscription = self.node.create_subscription(
            NavSatFix,
            'gps/fix',
            self._gps_callback,
            10
        )
        
        self.wind_subscription = self.node.create_subscription(
            Float32,
            'wind/direction',
            self._wind_callback,
            10
        )
    
    def _gps_callback(self, msg: NavSatFix):
        """store latest gps data"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
    
    def _wind_callback(self, msg: Float32):
        """store latest wind direction"""
        self.wind_direction = msg.data
    
    def get_current_position(self) -> tuple[float, float]:
        """get latest position"""
        return (self.current_lat, self.current_lon)
    
    def get_wind_direction(self) -> float:
        """get latest wind direction"""
        return self.wind_direction

    def handle_rc(self):
        """standard rc control implementation for all events"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        # implement standard rc control logic here
        pass

class FleetRaceControl(EventControl):
    """pure rc control - no autonomous capability"""
    pass  # only uses the base rc control

class PrecisionNavigationControl(EventControl):
    def handle_autonomous(self):
        """precision navigation autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        
        # get next waypoint
        if self.current_waypoint_index < len(self.waypoints):
            target_waypoint = self.waypoints[self.current_waypoint_index]
            # implement precision navigation autonomous behavior
            pass

class StationKeepingControl(EventControl):
    def handle_autonomous(self):
        """station keeping autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        
        # get station keeping target point
        if self.current_waypoint_index < len(self.waypoints):
            station_point = self.waypoints[self.current_waypoint_index]
            # implement station keeping autonomous behavior
            pass

class EnduranceControl(EventControl):
    def handle_autonomous(self):
        """endurance autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        
        # get next waypoint in endurance course
        if self.current_waypoint_index < len(self.waypoints):
            target_waypoint = self.waypoints[self.current_waypoint_index]
            # implement endurance autonomous behavior
            pass

class PayloadControl(EventControl):
    def handle_autonomous(self):
        """payload delivery autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        
        # get next waypoint for payload delivery
        if self.current_waypoint_index < len(self.waypoints):
            target_waypoint = self.waypoints[self.current_waypoint_index]
            # implement payload delivery autonomous behavior
            pass

class DeveloperControl(EventControl):
    def handle_autonomous(self):
        """developer testing autonomous control"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        
        # get next waypoint for testing
        if self.current_waypoint_index < len(self.waypoints):
            target_waypoint = self.waypoints[self.current_waypoint_index]
            # implement developer testing autonomous behavior
            pass

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