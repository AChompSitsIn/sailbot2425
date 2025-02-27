#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
import serial
import threading
import time
from datetime import datetime

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        self.declare_parameter('port', '/dev/ttyACM0') 
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('update_rate', 1.0) 
        
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        
        self.navsat_publisher = self.create_publisher(
            NavSatFix,
            'gps/fix',
            10
        )
        
        self.speed_publisher = self.create_publisher(
            Float64,
            'gps/speed',
            10
        )
        
        self.time_publisher = self.create_publisher(
            String,
            'gps/time',
            10
        )
        
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.speed = 0.0
        self.course = 0.0
        self.satellites = 0
        self.hdop = 0.0
        self.time_str = ""
        self.date_str = ""
        self.valid_fix = False

        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to GPS on {self.port} at {self.baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to GPS: {e}')
            self.serial_conn = None

        self.timer = self.create_timer(1.0/self.update_rate, self.timer_callback)

        self.running = True
        self.gps_thread = threading.Thread(target=self.read_gps_data)
        self.gps_thread.daemon = True
        self.gps_thread.start()
        
        self.get_logger().info('GPS Node initialized')
    
    def parse_time(self, time_str):
        """parse time from NMEA sentence"""
        try:
            time_obj = datetime.strptime(time_str, '%H%M%S.%f')
            return time_obj.strftime('%H:%M:%S')
        except ValueError:
            self.get_logger().warning('Invalid time format')
            return ""
    
    def parse_coordinates(self, lat_str, lat_dir, lon_str, lon_dir):
        """parse latitude and longitude from NMEA sentence"""
        try:
            lat = float(lat_str[:2]) + float(lat_str[2:]) / 60
            lon = float(lon_str[:3]) + float(lon_str[3:]) / 60
            
            if lat_dir == 'S':
                lat = -lat
            if lon_dir == 'W':
                lon = -lon
                
            return lat, lon
        except ValueError as e:
            self.get_logger().warning(f'Error parsing coordinates: {e}')
            return 0.0, 0.0
    
    def parse_gngga(self, sentence):
        """parse GGA message for satellites, altitude, etc."""
        parts = sentence.split(',')
        try:
            if len(parts) >= 15:
                # check if we have a fix (0 = no fix)
                fix_quality = int(parts[6])
                if fix_quality == 0:
                    self.valid_fix = False
                    return False
                
                self.altitude = float(parts[9]) if parts[9] else 0.0
                self.satellites = int(parts[7]) if parts[7] else 0
                self.hdop = float(parts[8]) if parts[8] else 0.0
                self.valid_fix = True
                
                if parts[2] and parts[4]:
                    lat, lon = self.parse_coordinates(parts[2], parts[3], parts[4], parts[5])
                    self.latitude = lat
                    self.longitude = lon
                
                return True
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f'Error parsing GGA: {e}')
        
        return False
    
    def parse_gnrmc(self, sentence):
        """Parse RMC message for speed, course, etc."""
        parts = sentence.split(',')
        try:
            if len(parts) >= 12:
                # check status (A=active, V=void)
                status = parts[2].upper()
                if status != 'A':
                    self.valid_fix = False
                    return False
                
                if parts[1]:
                    self.time_str = self.parse_time(parts[1])
                
                if parts[9] and len(parts[9]) >= 6:
                    day = parts[9][0:2]
                    month = parts[9][2:4]
                    year = "20" + parts[9][4:6]  # this assumes its the 21st century cause obviously
                    self.date_str = f"{year}-{month}-{day}"
 
                if parts[3] and parts[5]:
                    lat, lon = self.parse_coordinates(parts[3], parts[4], parts[5], parts[6])
                    self.latitude = lat
                    self.longitude = lon
                
                if parts[7]:
                    self.speed = float(parts[7]) * 0.514444  # convert knots to m/s for testing
                
                if parts[8]:
                    self.course = float(parts[8])
                
                self.valid_fix = True
                return True
                
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f'Error parsing RMC: {e}')
        
        return False
    
    def read_gps_data(self):
        """background thread to read data from gps"""
        if not self.serial_conn:
            self.get_logger().error('No serial connection available')
            return
            
        while self.running:
            try:
                line = self.serial_conn.readline().decode('ascii', errors='replace').strip()

                if not line:
                    continue

                if line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                    self.parse_gnrmc(line)
                elif line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                    self.parse_gngga(line)
                
            except Exception as e:
                self.get_logger().error(f'Error reading GPS data: {e}')
                time.sleep(1.0)
    
    def timer_callback(self):
        """publish gps data at regular intervals"""
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.header.frame_id = "gps_link"
        navsat_msg.latitude = self.latitude
        navsat_msg.longitude = self.longitude
        navsat_msg.altitude = self.altitude
        
        # set covariance to unknown if we don't have a fix
        if self.valid_fix:
            navsat_msg.status.status = 0  # STATUS_FIX
            navsat_msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
            navsat_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        else:
            navsat_msg.status.status = -1  # STATUS_NO_FIX
            navsat_msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
        
        self.navsat_publisher.publish(navsat_msg)
        
        # publish speed
        speed_msg = Float64()
        speed_msg.data = self.speed
        self.speed_publisher.publish(speed_msg)
        
        # publish time
        time_msg = String()
        time_msg.data = f"{self.date_str} {self.time_str}" if self.date_str and self.time_str else ""
        self.time_publisher.publish(time_msg)
        
        # log GPS data so i can debug
        self.get_logger().info(f'Published GPS data: lat={self.latitude:.6f}, lon={self.longitude:.6f}, ' 
                              f'speed={self.speed:.2f} m/s, sats={self.satellites}')
    
    def destroy_node(self):
        """clean up resources when node is shutdown"""
        self.running = False
        if hasattr(self, 'gps_thread') and self.gps_thread.is_alive():
            self.gps_thread.join(timeout=1.0)
        if hasattr(self, 'serial_conn') and self.serial_conn:
            self.serial_conn.close()
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

if __name__ == '__main__':
    main()