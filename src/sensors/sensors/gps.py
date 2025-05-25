#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String, Int32
import serial
import threading
import time
import re

# ANSI color codes from your paste.txt
class Colors:
    RESET = "\033[0m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"
    RED = "\033[91m"

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('update_rate', 1.0)
        
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Publishers
        self.navsat_publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.speed_publisher = self.create_publisher(Float64, 'gps/speed', 10)
        self.fix_publisher = self.create_publisher(Int32, 'gps/fix_quality', 10)
        
        # Initialize serial connection
        self.serial_conn = self.initialize_uart(self.port, self.baud_rate)
        
        # Initialize data variables
        self.lat = 0.0
        self.lon = 0.0
        self.speed = 0.0
        self.fix_quality = 0
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_data)
        
        # Start GPS reading thread
        self.running = True
        self.gps_thread = threading.Thread(target=self.read_gps_data)
        self.gps_thread.daemon = True
        self.gps_thread.start()
    
    def initialize_uart(self, port, baudrate, timeout=1):
        try:
            uart = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            self.get_logger().info(f"{Colors.GREEN}Connected to GPS on {port} at {baudrate} baud.{Colors.RESET}")
            return uart
        except Exception as e:
            self.get_logger().error(f"{Colors.RED}Error initializing UART: {str(e)}{Colors.RESET}")
            return None
    
    def is_valid_nmea(self, sentence):
        pattern = r"^\$[A-Z]{5},.*\*[0-9A-F]{2}$"
        return re.match(pattern, sentence)
    
    def parse_nmea_data(self, nmea_sentence):
        parts = nmea_sentence.split(',')

        if nmea_sentence.startswith('$GNGGA'):
            # Fix quality: 0 = invalid, 1 = GPS, 2 = DGPS, 4 = RTK Fixed, 5 = RTK Float
            fix_map = {
                '0': f"{Colors.RED}Invalid{Colors.RESET}",
                '1': f"{Colors.YELLOW}3D GPS Fix{Colors.RESET}",
                '2': f"{Colors.CYAN}DGPS Fix{Colors.RESET}",
                '4': f"{Colors.GREEN}RTK Fixed{Colors.RESET}",
                '5': f"{Colors.GREEN}RTK Float{Colors.RESET}"
            }
            fix_quality = parts[6] if len(parts) > 6 else '0'
            self.fix_quality = int(fix_quality) if fix_quality.isdigit() else 0
            fix_status = fix_map.get(fix_quality, f"{Colors.RED}Unknown{Colors.RESET}")

            # Position
            try:
                lat_raw = parts[2]
                lat_dir = parts[3]
                lon_raw = parts[4]
                lon_dir = parts[5]

                if lat_raw and lon_raw:
                    lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60
                    lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60
                    if lat_dir == 'S':
                        lat = -lat
                    if lon_dir == 'W':
                        lon = -lon

                    self.lat = lat
                    self.lon = lon
                    self.get_logger().debug(f"{Colors.CYAN}Position:{Colors.RESET} Lat: {lat:.6f}°, Lon: {lon:.6f}°")
                    self.get_logger().debug(f"{Colors.CYAN}Fix Status:{Colors.RESET} {fix_status}")
            except Exception:
                self.get_logger().warn(f"{Colors.RED}Error parsing position{Colors.RESET}")

        elif nmea_sentence.startswith('$GNVTG'):
            try:
                speed_knots = float(parts[5]) if len(parts) > 5 and parts[5] else 0
                speed_kmph = float(parts[7]) if len(parts) > 7 and parts[7] else 0
                self.speed = speed_kmph / 3.6  # Convert km/h to m/s
                self.get_logger().debug(f"{Colors.YELLOW}Speed:{Colors.RESET} {speed_kmph:.2f} km/h | {speed_knots:.2f} knots")
            except:
                self.get_logger().warn(f"{Colors.RED}Error parsing speed{Colors.RESET}")

    def read_gps_data(self):
        if not self.serial_conn:
            return
            
        try:
            while self.running:
                line = self.serial_conn.readline().decode(errors='ignore').strip()
                if self.is_valid_nmea(line):
                    self.parse_nmea_data(line)
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.get_logger().info(f"\n{Colors.YELLOW}Exiting...{Colors.RESET}")
    
    def publish_data(self):
        # Publish NavSatFix message
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.latitude = self.lat
        msg.longitude = self.lon
        msg.status.status = -1 if self.fix_quality == 0 else 0  # -1 is no fix, 0 is fix
        self.navsat_publisher.publish(msg)
        
        # Publish speed
        speed_msg = Float64()
        speed_msg.data = self.speed
        self.speed_publisher.publish(speed_msg)
        
        # Publish fix quality
        fix_msg = Int32()
        fix_msg.data = self.fix_quality
        self.fix_publisher.publish(fix_msg)
        
        # Log position if we have a fix
        if self.fix_quality > 0:
            self.get_logger().info(f"Position: Lat={self.lat:.6f}, Lon={self.lon:.6f}, Speed={self.speed:.2f}m/s, Fix={self.fix_quality}")
    
    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial_conn') and self.serial_conn:
            self.serial_conn.close()
            self.get_logger().info(f"{Colors.RED}UART connection closed.{Colors.RESET}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
