#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import serial
import struct
import threading
import time
import json

class RadioCommNode(Node):
    """
    ROS Node for radio communication between the sailbot and a remote control station.
    Receives commands from the remote station and publishes them to the 'radio_commands' topic.
    Sends status updates back to the remote station when requested.
    """
    def __init__(self):
        super().__init__('radio_comm_node')
        
        # Parameters for serial connection
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
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
        
        # Initialize status message storage
        self.latest_status = None
        
        # Create thread for receiving commands
        self.running = True
        self.receiver_thread = threading.Thread(target=self.receive_commands)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        # Timer for periodic initial message sending
        self.create_timer(5.0, self.send_initial_message)
        
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
                                self.send_status_update_simple()
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
        """Store latest status update from the boat system"""
        self.latest_status = msg.data
    
    def send_status_update_simple(self):
        """Send a simple status update message to the remote station"""
        if not self.serial_conn:
            return
            
        try:
            # For now, just send a simple text message
            status_text = "Status update complete"
            
            # Calculate length of data
            data_length = len(status_text)
            
            # Send header (0xD0), length, and data
            header = bytearray([0xD0, data_length])
            self.serial_conn.write(header)
            self.serial_conn.write(status_text.encode('utf-8'))
            
            # Log status update
            self.get_logger().info(f'Sent simple status update')
                
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