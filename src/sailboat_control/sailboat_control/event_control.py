from abc import ABC, abstractmethod
from .boat import ControlMode
from path_planning.path_planning.waypoint import Waypoint
from path_planning.path_planning.leg import leg
from typing import List, Optional

class EventControl(ABC):
    """base class for event control"""
    def __init__(self, waypoints: List[Waypoint]):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
    # standard rc control shared by all events
        
    def handle_rc(self):
        """standard rc control implementation for all events"""
        # implement standard rc control logic here
        # this will be the same for every event type
        pass

class FleetRaceControl(EventControl):
    """pure rc control - no autonomous capability"""
    pass  # only uses the base rc control

class PrecisionNavigationControl(EventControl):
    def handle_autonomous(self):
        """precision navigation autonomous control"""
        # implement precision autonomous behavior
        pass

class StationKeepingControl(EventControl):
    def handle_autonomous(self):
        """station keeping autonomous control"""
        # implement station keeping autonomous behavior
        pass

class EnduranceControl(EventControl):
    def handle_autonomous(self):
        """endurance autonomous control"""
        # implement endurance autonomous behavior
        pass

class PayloadControl(EventControl):
    def handle_autonomous(self):
        """payload delivery autonomous control"""
        # implement payload autonomous behavior
        pass

class DeveloperControl(EventControl):
    def handle_autonomous(self):
        """developer testing autonomous control"""
        # implement developer autonomous behavior
        pass

def create_event_control(event_type: str, waypoints: List[Waypoint]) -> EventControl:
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
        return control_class(waypoints)
    else:
        raise ValueError(f"Unknown event type: {event_type}")