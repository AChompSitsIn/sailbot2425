import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from .boat import Boat, ControlMode

class StateManagementNode(Node):
    def __init__(self):
        """initialize the state management ROS node"""
        super().__init__('state_management_node')
        
        self.radio_subscriber = self.create_subscription( # add actual radio handling later
            Int32,
            'radio_commands',
            self.radio_callback,
            10
        )
        
        self.boat = Boat("developer_mode") # boat initially starts in developer mode under RC control
        self.boat.start_event()
        
        self.command_handlers = {
            0: self._handle_rc_control,
            1: self._handle_start_autonomous,
            2: self._handle_rc_interrupt,
            3: self._handle_resume_autonomous,
            4: self._handle_emergency_stop
        }
        
        self.get_logger().info("State management node initialized")
    
    def radio_callback(self, msg: Int32):
        """handle incoming radio commands"""
        command = msg.data
        handler = self.command_handlers.get(command)
        
        if handler:
            handler()
            self._log_state_change()
        else:
            self.get_logger().warn(f"Unknown command received: {command}")
    
    def _handle_rc_control(self):
        """handle RC control command"""
        self.boat.state.control_mode = ControlMode.RC
        self.get_logger().info("Switched to RC control")
        # Call RC control method
        self.boat.event_control.handle_rc()

    def _handle_start_autonomous(self):
        """handle start autonomous command"""
        self.boat.start_autonomous()
        # If event supports autonomous, call its autonomous control
        if hasattr(self.boat.event_control, 'handle_autonomous'):
            self.boat.event_control.handle_autonomous()

    def _handle_rc_interrupt(self):
        """handle RC interrupt command"""
        last_waypoint = self.boat.handle_rc_interrupt()
        if last_waypoint:
            self.get_logger().info(f"RC interrupt - last waypoint: {last_waypoint}")
        # Switch to RC control during interrupt
        self.boat.event_control.handle_rc()s

    def _handle_resume_autonomous(self):
        """handle resume autonomous command"""
        self.boat.resume_autonomous()
        # Resume autonomous control if event supports it
        if hasattr(self.boat.event_control, 'handle_autonomous'):
            self.boat.event_control.handle_autonomous()

    def _handle_emergency_stop(self):
        """handle emergency stop command"""
        self.boat.state.control_mode = ControlMode.RC
        self.boat.state.is_event_active = False
        self.get_logger().warn("Emergency stop activated")
        # Force RC control
        self.boat.event_control.handle_rc()
    
    def _log_state_change(self):
        """Log current state after changes"""
        status = self.boat.get_system_status()
        self.get_logger().info(
            f"State update - mode: {status['control_mode']}, "
            f"event: {status['event_type']}, "
            f"autonomous enabled: {status['autonomous_enabled']}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StateManagementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()