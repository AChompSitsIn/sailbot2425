#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import NavSatFix
import serial
import struct
import threading
import time
import json

class RadioCommNode(Node):
    """
    ROS Node for radio communication between the sailbot and a remote control station.
    Receives commands from the remote station and publishes them to the 'radio_commands' topic.
    Sends status updates back to the remote station when requested, using simple text format.
    """
    def __init__(self):
        super().__init__('radio_comm_node')
        
        # Parameters for serial connection
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('status_interval', 5.0)  # Interval in seconds for auto status updates
        
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.status_interval = self.get_parameter('status_interval').value
        
        # Command publisher
        self.command_publisher = self.create_publisher(
            Int32,
            'radio_commands',
            10
        )
        
        # Subscribe to status info
        self.status_subscription = self.create_subscription(
            String,
            'boat_status',
            self.status_callback,
            10
        )
        
        # Subscribe to GPS data
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )
        
        # Subscribe to GPS speed
        self.speed_subscription = self.create_subscription(
            Float64,
            'gps/speed',
            self.speed_callback,
            10
        )
        
        # Subscribe to GPS fix quality
        self.fix_quality_subscription = self.create_subscription(
            Int32,
            'gps/fix_quality',
            self.fix_quality_callback,
            10
        )
        
        # Subscribe to wind direction
        self.wind_subscription = self.create_subscription(
            Float32,
            'wind/direction',
            self.wind_callback,
            10
        )
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to radio on {self.port} at {self.baud_rate} baud')
            
            # Send initial connection message
            self.send_initial_message()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to radio: {e}')
            self.serial_conn = None
        
        # Initialize sensor data storage
        self.latest_gps = {
            "latitude": 0.0,
            "longitude": 0.0,
            "fix_quality": 0,
            "speed": 0.0,
            "timestamp": 0
        }
        self.latest_wind_direction = 0.0
        
        # Status information - stored in simple format
        self.control_mode = "rc"
        self.event_type = "developer_mode"
        self.current_waypoint = None
        self.total_waypoints = 0
        self.last_completed_waypoint = None
        
        # Create thread for receiving commands
        self.running = True
        self.receiver_thread = threading.Thread(target=self.receive_commands)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        # Timer for periodic initial message sending
        self.create_timer(5.0, self.send_initial_message)
        
        # Timer for periodic status updates
        self.create_timer(self.status_interval, self.send_periodic_status)
        
        self.get_logger().info('Radio Communication Node initialized')
    
    def send_initial_message(self):
        """Send initial message to establish connection with the remote terminal"""
        if not self.serial_conn:
            return
            
        try:
            # Simple initial message: 0xB0 (header) + 0x55 + 0xAA (magic bytes)
            init_msg = bytearray([0xB0, 0x55, 0xAA])
            self.serial_conn.write(init_msg)
            self.get_logger().debug('Sent initial connection message')
        except Exception as e:
            self.get_logger().error(f'Error sending initial message: {e}')
    
    def gps_callback(self, msg: NavSatFix):
        """Store latest GPS position data"""
        self.latest_gps["latitude"] = msg.latitude
        self.latest_gps["longitude"] = msg.longitude
        self.latest_gps["timestamp"] = self.get_clock().now().to_msg().sec
    
    def speed_callback(self, msg: Float64):
        """Store latest GPS speed data"""
        self.latest_gps["speed"] = msg.data
    
    def fix_quality_callback(self, msg: Int32):
        """Store latest GPS fix quality data"""
        self.latest_gps["fix_quality"] = msg.data
    
    def wind_callback(self, msg: Float32):
        """Store latest wind direction data"""
        self.latest_wind_direction = msg.data
        self.get_logger().debug(f"Received wind direction: {msg.data:.1f}°")
    
    def receive_commands(self):
        """Background thread to receive commands from the radio"""
        if not self.serial_conn:
            self.get_logger().error('No serial connection available for receiving')
            return
        
        buffer = bytearray()
        
        while self.running:
            try:
                # Check if data is available
                if self.serial_conn.in_waiting > 0:
                    # Read one byte
                    buffer += self.serial_conn.read(1)
                    
                    # Check for command header (0xC0)
                    if len(buffer) == 1 and buffer[0] != 0xC0:
                        buffer = bytearray()
                        continue
                    
                    # If we have at least 3 bytes (header + command byte + checksum)
                    if len(buffer) >= 3:
                        # Validate checksum (simple XOR)
                        if buffer[1] ^ 0xFF == buffer[2]:
                            # Extract command
                            command = buffer[1]
                            
                            # Send immediate acknowledgment
                            ack_msg = bytearray([0xA0, command, command ^ 0xFF])
                            self.serial_conn.write(ack_msg)
                            
                            # Special handling for status request command (9)
                            if command == 9:
                                self.get_logger().info('Status update requested')
                                self.send_status_update()
                            else:
                                # Process other commands
                                msg = Int32()
                                msg.data = command
                                self.command_publisher.publish(msg)
                                
                                # Log received command
                                self.get_logger().info(f'Received command: {command}')
                        else:
                            self.get_logger().warning('Invalid checksum in received command')
                        
                        # Reset buffer for next command
                        buffer = bytearray()
                else:
                    # No data available, sleep briefly
                    time.sleep(0.01)
                    
            except Exception as e:
                self.get_logger().error(f'Error receiving command: {e}')
                buffer = bytearray()
                time.sleep(0.1)
    
    def status_callback(self, msg):
        """Process status update from the boat system"""
        try:
            # Parse the JSON status
            status_data = json.loads(msg.data)
            
            # Extract the values we need in simple format
            self.control_mode = status_data.get("control_mode", "rc")
            self.event_type = status_data.get("event_type", "developer_mode")
            
            # Handle waypoint information
            if "current_waypoint" in status_data and status_data["current_waypoint"]:
                self.current_waypoint = status_data["current_waypoint"]
            
            if "last_completed_waypoint" in status_data and status_data["last_completed_waypoint"]:
                self.last_completed_waypoint = status_data["last_completed_waypoint"]
            
            self.total_waypoints = status_data.get("total_waypoints", 0)
            
            # Log that we received a status update
            self.get_logger().info(
                f"Received status update - mode: {self.control_mode}, "
                f"event: {self.event_type}"
            )
            
            # Send a status update when mode changes
            self.send_status_update()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")
    
    def send_periodic_status(self):
        """Send periodic status updates to remote terminal"""
        self.send_status_update()
    
    def send_status_update(self):
        """Send status update using plain text format"""
        if not self.serial_conn:
            return
            
        try:
            # Send GPS data in simple text format
            # Format: GPS,lat,lon,speed,fix_quality
            gps_message = f"GPS,{self.latest_gps['latitude']:.6f},{self.latest_gps['longitude']:.6f},{self.latest_gps['speed']:.2f},{self.latest_gps['fix_quality']}\n"
            self.serial_conn.write(gps_message.encode('utf-8'))
            self.get_logger().info(f"Sent GPS data: {gps_message.strip()}")
            
            # Send wind direction data
            # Format: WIND,direction
            wind_message = f"WIND,{self.latest_wind_direction:.1f}\n"
            self.serial_conn.write(wind_message.encode('utf-8'))
            self.get_logger().info(f"Sent wind data: {wind_message.strip()}")
            
            # Send boat status with actual control mode and event type
            # Format: STATUS,control_mode,event_type
            status_message = f"STATUS,{self.control_mode},{self.event_type}\n"
            self.serial_conn.write(status_message.encode('utf-8'))
            self.get_logger().info(f"Sent boat status: {status_message.strip()}")
            
            # Send waypoint info if available
            if self.current_waypoint:
                try:
                    current_wp_info = None
                    if isinstance(self.current_waypoint, dict):
                        # If current_waypoint is already a dict
                        current_wp_info = str(self.current_waypoint.get("lat", 0)) + "," + str(self.current_waypoint.get("long", 0))
                    else:
                        # If it's something else (assuming it's an object with lat/long attributes)
                        try:
                            current_wp_info = str(getattr(self.current_waypoint, "lat", 0)) + "," + str(getattr(self.current_waypoint, "long", 0))
                        except:
                            current_wp_info = "unknown"
                    
                    # Format: WAYPOINT,info,total
                    waypoint_message = f"WAYPOINT,{current_wp_info},{self.total_waypoints}\n"
                    self.serial_conn.write(waypoint_message.encode('utf-8'))
                    self.get_logger().info(f"Sent waypoint info: {waypoint_message.strip()}")
                except Exception as e:
                    self.get_logger().warning(f"Error formatting waypoint data: {e}")
                
        except Exception as e:
            self.get_logger().error(f'Error sending status update: {e}')
    
    def destroy_node(self):
        """Clean up resources when node is shutdown"""
        self.running = False
        if hasattr(self, 'receiver_thread') and self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)
        if hasattr(self, 'serial_conn') and self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RadioCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()