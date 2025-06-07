from enum import Enum
from typing import List, Optional
from rclpy.node import Node
from path_planning.path_planning.waypoint import Waypoint
from .common import ControlMode  # Changed from local definition
from .events import F, Pr, S, E, P, D
from .event_control import create_event_control

class BoatState:
    def __init__(self):
        self.control_mode: ControlMode = ControlMode.RC
        self.current_waypoint: Optional[Waypoint] = None
        self.last_completed_waypoint: Optional[Waypoint] = None
        self.is_event_active: bool = False
        self.autonomous_enabled: bool = False
        self.rc_enabled_after_autonomous: bool = False  # Track if RC was enabled after autonomous mode

class Boat:
    def __init__(self, event_type: str, node: Node):
        self.event_type = event_type
        self.node = node  # store ros node for sensor access
        self.autonomous_system_initialized = False
        self.state = BoatState()
        self.current_event = None
        self.waypoints: List[Waypoint] = []
        self.event_control = None  # will hold the event control instance

    def pick_event(self):
        """pick and initialize event based on event type"""
        event_classes = {
            "fleet_race": F,
            "precision_navigation": Pr,
            "station_keeping": S,
            "endurance": E,
            "payload": P,
            "developer_mode": D
        }
        
        event_class = event_classes.get(self.event_type.lower())
        if event_class:
            self.current_event = event_class()
            self.current_event.initialize_event(self)
            # create the appropriate event control with ros node
            self.event_control = create_event_control(
                self.event_type,
                self.waypoints,
                self.node
            )

    def start_event(self) -> None:
        """initialize and start event"""
        self.pick_event()
        if self.current_event:
            self.state.is_event_active = True
            if self.state.autonomous_enabled:
                self.state.control_mode = ControlMode.RC

    def start_autonomous(self) -> None:
        """start autonomous control if enabled"""
        if not self.state.is_event_active:
            return
        
        if not self.state.autonomous_enabled:
            return

        self.state.control_mode = ControlMode.AUTONOMOUS
        # Log the change
        self.node.get_logger().info(f"Autonomous mode started for {self.event_type}")

    def handle_rc_interrupt(self) -> Optional[Waypoint]:
        """handle RC interrupt for autonomous events - switches to RC mode permanently"""
        if not self.state.autonomous_enabled:
            return None
            
        if self.state.control_mode != ControlMode.RC:
            self.state.control_mode = ControlMode.RC
            self.state.rc_enabled_after_autonomous = True  # Mark that RC was enabled after autonomous
            # Log the change
            self.node.get_logger().info("RC interrupt activated - switching to RC mode (permanent)")
            return self.state.last_completed_waypoint
        return None

    def resume_autonomous(self) -> None:
        """resume autonomous control from current position"""
        if not self.state.autonomous_enabled:
            return
            
        # Prevent resuming autonomous if RC was enabled after autonomous mode
        if self.state.rc_enabled_after_autonomous:
            self.node.get_logger().warn("Cannot resume autonomous - RC was enabled after autonomous mode")
            return
            
        if self.state.control_mode == ControlMode.RC:
            self.state.control_mode = ControlMode.AUTONOMOUS
            # Log the change
            self.node.get_logger().info("Autonomous control resumed")

    def get_system_status(self) -> dict:
        """get current system status"""
        waypoint_data = None
        if self.state.current_waypoint:
            waypoint_data = {
                "lat": self.state.current_waypoint.lat,
                "long": self.state.current_waypoint.long,
                "marked": self.state.current_waypoint.marked,
                "n_passed": self.state.current_waypoint.n_passed
            }
            
        last_waypoint_data = None
        if self.state.last_completed_waypoint:
            last_waypoint_data = {
                "lat": self.state.last_completed_waypoint.lat,
                "long": self.state.last_completed_waypoint.long,
                "marked": self.state.last_completed_waypoint.marked,
                "n_passed": self.state.last_completed_waypoint.n_passed
            }
            
        return {
            "event_type": self.event_type,
            "control_mode": self.state.control_mode.value,
            "autonomous_enabled": self.state.autonomous_enabled,
            "event_active": self.state.is_event_active,
            "current_waypoint": waypoint_data,
            "last_completed_waypoint": last_waypoint_data,
            "total_waypoints": len(self.waypoints)
        }