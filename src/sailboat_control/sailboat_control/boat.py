from enum import Enum
from typing import List, Optional
from path_planning.path_planning.waypoint import Waypoint
from .events import F, Pr, S, E, P, D

class ControlMode(Enum):
    RC = "rc"                    
    AUTONOMOUS = "autonomous"     
    RC_INTERRUPT = "rc_interrupt" 

class BoatState:
    def __init__(self):
        self.control_mode: ControlMode = ControlMode.RC
        self.current_waypoint: Optional[Waypoint] = None
        self.last_completed_waypoint: Optional[Waypoint] = None
        self.is_event_active: bool = False
        self.autonomous_enabled: bool = False

class Boat:
    def __init__(self, event_type: str):
        self.event_type = event_type
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
            # create the appropriate event control
            self.event_control = create_event_control(self.event_type, self.waypoints)

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

    def handle_rc_interrupt(self) -> Optional[Waypoint]:
        """handle RC interrupt for autonomous events"""
        if not self.state.autonomous_enabled:
            return None
            
        if self.state.control_mode != ControlMode.RC_INTERRUPT:
            self.state.control_mode = ControlMode.RC_INTERRUPT
            return self.state.last_completed_waypoint
        return None

    def resume_autonomous(self) -> None:
        """resume autonomous control from current position"""
        if not self.state.autonomous_enabled:
            return
            
        if self.state.control_mode == ControlMode.RC_INTERRUPT:
            self.state.control_mode = ControlMode.AUTONOMOUS

    def get_system_status(self) -> dict:
        """get current system status"""
        return {
            "event_type": self.event_type,
            "control_mode": self.state.control_mode.value,
            "autonomous_enabled": self.state.autonomous_enabled,
            "event_active": self.state.is_event_active,
            "current_waypoint": self.state.current_waypoint,
            "last_completed_waypoint": self.state.last_completed_waypoint,
            "total_waypoints": len(self.waypoints)
        }