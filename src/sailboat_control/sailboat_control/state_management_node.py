#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from .boat import Boat, ControlMode
import json

class StateManagementNode(Node):
    def __init__(self):
        """initialize the state management ROS node"""
        super().__init__('state_management_node')

        # Radio command subscription
        self.radio_subscriber = self.create_subscription(
            Int32,
            'radio_commands',
            self.radio_callback,
            10
        )

        # Add status publisher for boat status
        self.status_publisher = self.create_publisher(
            String,
            'boat_status',
            10
        )

        # Create the boat instance and pass the node for sensor access
        self.boat = Boat("developer_mode", self)
        self.boat.start_event()

        # Command handler mapping
        self.command_handlers = {
            0: self._handle_rc_control,
            1: self._handle_start_autonomous,
            2: self._handle_rc_interrupt,
            3: self._handle_resume_autonomous,
            4: self._handle_emergency_stop
        }

        # Publish initial status
        self.publish_status()

        # Create a timer to periodically publish status (every 5 seconds)
        self.create_timer(5.0, self.publish_status)

        self.get_logger().info("State management node initialized")

    def radio_callback(self, msg: Int32):
        """handle incoming radio commands"""
        command = msg.data
        handler = self.command_handlers.get(command)

        if handler:
            handler()
            self._log_state_change()
            # Publish status update immediately after state change
            self.publish_status()
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def _handle_rc_control(self):
        """handle RC control command"""
        self.boat.state.control_mode = ControlMode.RC
        self.get_logger().info("Switched to RC control")
        # Call RC control method
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.handle_rc()

    def _handle_start_autonomous(self):
        """handle start autonomous command"""
        self.boat.start_autonomous()
        # If event supports autonomous, call its autonomous control
        if hasattr(self.boat, 'event_control') and self.boat.event_control and \
           hasattr(self.boat.event_control, 'handle_autonomous'):
            self.boat.event_control.handle_autonomous()

    def _handle_rc_interrupt(self):
        """handle RC interrupt command"""
        last_waypoint = self.boat.handle_rc_interrupt()
        if last_waypoint:
            self.get_logger().info(f"RC interrupt - last waypoint: {last_waypoint}")
        # Switch to RC control during interrupt
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.handle_rc()
            self.boat.event_control.disable_autonomous_navigation() # should enable rc flag (no nav node control)

    def _handle_resume_autonomous(self):
        """handle resume autonomous command"""
        self.boat.resume_autonomous()
        # Resume autonomous control if event supports it
        if hasattr(self.boat, 'event_control') and self.boat.event_control and \
           hasattr(self.boat.event_control, 'handle_autonomous'):
            self.boat.event_control.handle_autonomous()

    def _handle_emergency_stop(self):
        """handle emergency stop command"""
        self.boat.state.control_mode = ControlMode.RC
        self.boat.state.is_event_active = False
        self.get_logger().warn("Emergency stop activated")
        # Force RC control
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.handle_rc()

    def _log_state_change(self):
        """Log current state after changes"""
        status = self.boat.get_system_status()
        self.get_logger().info(
            f"State update - mode: {status['control_mode']}, "
            f"event: {status['event_type']}, "
            f"autonomous enabled: {status['autonomous_enabled']}"
        )

    def publish_status(self):
        """Publish boat status for radio communication"""
        status = self.boat.get_system_status()

        # Add GPS and wind data if available
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            try:
                current_pos = self.boat.event_control.get_current_position()
                status['position'] = current_pos
                status['speed'] = self.boat.event_control.get_current_speed()
                status['wind_direction'] = self.boat.event_control.get_wind_direction()
            except AttributeError:
                # Handle case where event_control doesn't have all methods
                self.get_logger().debug("Some event_control methods not available")

        # Convert to JSON string
        status_json = json.dumps(status)

        # Create and publish message
        msg = String()
        msg.data = status_json
        self.status_publisher.publish(msg)

        # Log status update
        self.get_logger().debug(f"Published status update: {status_json}")

def main(args=None):
    rclpy.init(args=args)
    node = StateManagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
