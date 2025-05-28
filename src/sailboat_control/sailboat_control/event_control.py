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
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str = "unknown"):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.node = node  # ros2 node for subscriptions
        self.event_type = event_type # Store event type for specific logic
        self.leg_calculator = Leg() # Initialize Leg calculator
        self.navigation_enabled_by_event_control = False # Track if this instance enabled nav

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
            # This logic runs continuously, even if in RC mode, so BuoyDetector state is up-to-date.
            current_buoy_object = self.buoy_detector.get_current_buoy() # Renamed for clarity
            if current_buoy_object is not None:
                try:
                    current_reached_idx = self.waypoints.index(current_buoy_object)
                    
                    # Determine next logical waypoint index, handling looping for endurance
                    new_potential_idx = current_reached_idx + 1
                    is_looping_event = "endurance" in self.event_type.lower() # Example check

                    if is_looping_event and self.waypoints:
                        self.current_waypoint_index = new_potential_idx % len(self.waypoints)
                    elif new_potential_idx < len(self.waypoints):
                        self.current_waypoint_index = new_potential_idx
                    else: # End of a non-looping event sequence
                        self.current_waypoint_index = new_potential_idx # Will be out of bounds

                    next_target_waypoint = self.get_next_waypoint() # Uses the updated index

                    if next_target_waypoint:
                        self.node.get_logger().info(
                            f"Next target (major event waypoint): {next_target_waypoint.lat}, {next_target_waypoint.long}"
                        )
                        # If navigation was previously enabled by this EventControl instance,
                        # and we assume StateManagementNode handles global mode,
                        # then we can send the next waypoint.
                        # A more robust system would check global boat_state.control_mode here.
                        if self.navigation_enabled_by_event_control:
                             self.send_waypoint(next_target_waypoint)
                    else:
                        self.node.get_logger().info("All major event waypoints reached or event sequence complete.")
                        if self.navigation_enabled_by_event_control:
                            # self.disable_autonomous_navigation() # Consider if this should be here or handled by StateManagementNode
                            self.navigation_enabled_by_event_control = False # Mark as no longer controlling nav

                except ValueError:
                    self.node.get_logger().error("Current buoy from detector not found in master waypoints list!")
        
        # Log position updates for debugging (can be made less frequent if needed)
        # self.node.get_logger().debug(f"Position update: lat={self.current_lat:.6f}, lon={self.current_lon:.6f}")

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
        
        if 0 <= self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        
        # If index is out of bounds (e.g., end of a non-looping event)
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
        """Enable autonomous navigation via NavigationNode"""
        msg = Int32()
        msg.data = 1  # 1 = enable
        self.nav_command_publisher.publish(msg)
        self.navigation_enabled_by_event_control = True # Track that this instance enabled it
        self.node.get_logger().info("EventControl: Enabled autonomous navigation (sent command to NavigationNode)")
    
    def disable_autonomous_navigation(self):
        """Disable autonomous navigation via NavigationNode"""
        msg = Int32()
        msg.data = 0  # 0 = disable
        self.nav_command_publisher.publish(msg)
        self.navigation_enabled_by_event_control = False # Track that this instance disabled it
        self.node.get_logger().info("EventControl: Disabled autonomous navigation (sent command to NavigationNode)")
        
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
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"PrecisionNavigationControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """precision navigation autonomous control"""
        self.node.get_logger().info("PrecisionNavigationControl: Handling autonomous sequence...")
        
        # The _gps_callback should have already updated current_waypoint_index
        # based on any buoys passed, even during RC.
        # So, get_next_waypoint() should give us the correct current target.
        next_wp = self.get_next_waypoint()

        if next_wp:
            self.node.get_logger().info(f"PrecisionNavigationControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            self.enable_autonomous_navigation() 
            self.send_waypoint(next_wp)
        else:
            self.node.get_logger().info("PrecisionNavigationControl: All waypoints reached or no waypoints defined for this event.")
            # Optionally disable navigation if the event is truly over and doesn't loop
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class StationKeepingControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"StationKeepingControl initialized for event: {event_type}")
        # For station keeping, waypoints[0] is typically the center.
        # A radius parameter would also be needed for real station keeping.
        self.station_keeping_radius = 20.0 # Example radius in meters, should be configurable

    def handle_autonomous(self):
        """station keeping autonomous control"""
        self.node.get_logger().info("StationKeepingControl: Handling autonomous...")
        
        # The first waypoint is the center of the station-keeping zone.
        station_center_wp = self._get_waypoint_at_index(0) # Always target the first defined waypoint as center

        if station_center_wp:
            self.node.get_logger().info(f"StationKeepingControl: Target center at ({station_center_wp.lat}, {station_center_wp.long})")
            
            # For now, basic strategy: continuously tell NavigationNode to go to/hold at the center.
            # NavigationNode would need to be smart about "holding" vs just "reaching".
            # A more advanced StationKeepingControl would calculate micro-waypoints or adjust
            # parameters for NavigationNode.
            self.enable_autonomous_navigation()
            self.send_waypoint(station_center_wp)
            
            # Note: True station keeping (maintaining position within a radius) is complex.
            # This implementation just sends the center point. The NavigationNode
            # would need a "loiter" or "station keep" capability, or this node
            # would need to generate a series of small waypoints to simulate it.
            # For example, if NavigationNode only goes to a point and stops,
            # this class would need to detect drifting and resend the center point.
        else:
            self.node.get_logger().warning("StationKeepingControl: No center waypoint defined for station keeping.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class EnduranceControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"EnduranceControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """endurance autonomous control - waypoints should loop"""
        self.node.get_logger().info("EnduranceControl: Handling autonomous sequence...")
        
        # _gps_callback handles advancing current_waypoint_index, including looping
        # for "endurance" type events.
        next_wp = self.get_next_waypoint() # Will get waypoints[0] after last if looping is correct

        if next_wp:
            self.node.get_logger().info(f"EnduranceControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            self.enable_autonomous_navigation()
            self.send_waypoint(next_wp)
        else:
            # This case should ideally not be reached if waypoints are defined and looping works.
            self.node.get_logger().error("EnduranceControl: No next waypoint available, though endurance should loop. Check waypoint list and index logic.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class PayloadControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"PayloadControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """payload delivery autonomous control"""
        self.node.get_logger().info("PayloadControl: Handling autonomous sequence...")
        next_wp = self.get_next_waypoint()

        if next_wp:
            self.node.get_logger().info(f"PayloadControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            self.enable_autonomous_navigation()
            self.send_waypoint(next_wp)
            
            # Additional logic for payload deployment could be triggered here
            # by checking if the *current* waypoint (which is next_wp if it's the first in sequence,
            # or previous one if sequence is ongoing) has been reached and is the payload drop zone.
            # E.g., if self.buoy_detector.is_buoy_reached(specific_payload_waypoint):
            #    self.node.get_logger().info("Payload waypoint reached! Deploying payload (simulated).")
            #    # ... actual payload deployment command ...
        else:
            self.node.get_logger().info("PayloadControl: All waypoints reached or no waypoints defined.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class DeveloperControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str): # Added event_type
        super().__init__(waypoints, node, event_type) # Pass event_type
        self.node.get_logger().info(f"DeveloperControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """developer testing autonomous control"""
        self.node.get_logger().info("DeveloperControl: Handling autonomous...")
        # Get next waypoint. _gps_callback should keep current_waypoint_index updated.
        next_wp = self.get_next_waypoint()
        
        if next_wp:
            self.node.get_logger().info(f"DeveloperControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            # Enable autonomous navigation
            self.enable_autonomous_navigation()
            
            # Send waypoint to navigation system
            self.send_waypoint(next_wp)
        else:
            self.node.get_logger().warning("DeveloperControl: No waypoint available for navigation or sequence complete.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

def create_event_control(event_type: str, waypoints: List[Waypoint], node: Node) -> Optional[EventControl]:
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
        # Pass event_type to the constructor of the specific EventControl class
        return control_class(waypoints, node, event_type)
    else:
        node.get_logger().error(f"Unknown event type: {event_type}. Cannot create EventControl.")
        return None # Return None instead of raising ValueError to prevent node crash
