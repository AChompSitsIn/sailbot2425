#!/usr/bin/env python3
import serial
import struct
import threading
import time
import json
import os
import argparse
import curses
import sys
from datetime import datetime

class SailbotRemoteControl:
    """
    Remote control terminal interface for Sailbot
    Uses RFD900x radio modem to send commands and receive status updates
    """
    def __init__(self, port, baud_rate=57600):
        self.port = port
        self.baud_rate = baud_rate
        self.running = True
        self.latest_status = {}
        self.command_ack = {}  # To track command acknowledgments
        self.last_update_time = 0
        self.connection_healthy = False
        self.input_buffer = ""
        self.debug_messages = []
        
        # Command definitions
        self.commands = {
            '0': {"name": "RC Control", "code": 0, "description": "Switch to manual RC control mode"},
            '1': {"name": "Autonomous", "code": 1, "description": "Start autonomous navigation"},
            '2': {"name": "RC Interrupt", "code": 2, "description": "Temporarily interrupt autonomous mode"},
            '3': {"name": "Resume Auto", "code": 3, "description": "Resume autonomous mode after interruption"},
            '4': {"name": "EMERGENCY STOP", "code": 4, "description": "Emergency stop all operations"},
            '9': {"name": "Status Request", "code": 9, "description": "Request immediate status update"}
        }
        
        try:
            # Connect to serial port
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.add_debug_message(f"Connected to radio on {self.port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.add_debug_message(f"Failed to connect to radio: {e}")
            self.serial_conn = None
            sys.exit(1)
            
        # Start receiver thread
        self.receiver_thread = threading.Thread(target=self.receive_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
    
    def add_debug_message(self, message):
        """Add message to debug log with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.debug_messages.append(f"[{timestamp}] {message}")
        # Keep only last 100 messages
        if len(self.debug_messages) > 100:
            self.debug_messages = self.debug_messages[-100:]
    
    def send_command(self, command_code):
        """Send a command to the sailbot"""
        if not self.serial_conn:
            return False
            
        try:
            # Command format: Header byte (0xC0) + Command byte + Checksum byte (XOR with 0xFF)
            checksum = command_code ^ 0xFF
            command_bytes = bytearray([0xC0, command_code, checksum])
            self.serial_conn.write(command_bytes)
            
            # Update command acknowledgment status
            self.command_ack[command_code] = {"sent": time.time(), "ack": False}
            
            self.add_debug_message(f"Sent command: {command_code} - {self.get_command_name(command_code)}")
            
            return True
        except Exception as e:
            self.add_debug_message(f"Error sending command: {e}")
            return False
    
    def get_command_name(self, code):
        """Get command name from code"""
        for key, cmd in self.commands.items():
            if cmd["code"] == code:
                return cmd["name"]
        return "Unknown"
    
    def receive_data(self):
        """Background thread to receive data from the radio"""
        if not self.serial_conn:
            return
            
        buffer = bytearray()
        state = "IDLE"  # States: IDLE, READING_STATUS, READING_ACK
        status_length = 0
        
        while self.running:
            try:
                # Check if data is available
                if self.serial_conn.in_waiting > 0:
                    # Read one byte
                    byte_data = self.serial_conn.read(1)
                    buffer += byte_data
                    
                    if state == "IDLE":
                        # Check for packet headers
                        if buffer[0] == 0xD0:  # Status update header
                            state = "READING_STATUS"
                            if len(buffer) > 1:
                                status_length = buffer[1]
                        elif buffer[0] == 0xA0:  # Acknowledgment header
                            state = "READING_ACK"
                        elif buffer[0] == 0xB0:  # Initial connection message
                            state = "READING_INIT"
                        else:
                            # Unknown header, reset buffer
                            buffer = bytearray()
                            
                    elif state == "READING_STATUS":
                        # Wait until we have the length byte plus the full status data
                        if len(buffer) >= 2 and len(buffer) >= (2 + status_length):
                            # Extract status message
                            status_data = buffer[2:2+status_length].decode('utf-8')
                            self.add_debug_message(f"Received status: {status_data}")
                            self.last_update_time = time.time()
                            self.connection_healthy = True
                                
                            # Reset for next packet
                            buffer = bytearray()
                            state = "IDLE"
                    
                    elif state == "READING_INIT":
                        # Check for complete initial message (3 bytes)
                        if len(buffer) >= 3:
                            # Validate magic bytes
                            if buffer[1] == 0x55 and buffer[2] == 0xAA:
                                self.connection_healthy = True
                                self.last_update_time = time.time()
                                self.add_debug_message("Connection established with jetson!")
                            
                            # Reset for next packet
                            buffer = bytearray()
                            state = "IDLE"
                            
                    elif state == "READING_ACK":
                        # Wait until we have a complete ACK packet (3 bytes)
                        if len(buffer) >= 3:
                            # Validate checksum
                            if buffer[1] ^ 0xFF == buffer[2]:
                                # Mark command as acknowledged
                                command = buffer[1]
                                if command in self.command_ack:
                                    self.command_ack[command]["ack"] = True
                                    self.add_debug_message(f"Command acknowledged: {command} - {self.get_command_name(command)}")
                                    
                                    # Mark connection as healthy when we get any acknowledgment
                                    self.connection_healthy = True
                                    self.last_update_time = time.time()
                            
                            # Reset for next packet
                            buffer = bytearray()
                            state = "IDLE"
                    
                    # Prevent buffer overflow
                    if len(buffer) > 256:
                        buffer = bytearray()
                        state = "IDLE"
                else:
                    # No data available, sleep briefly
                    time.sleep(0.01)
                    
            except Exception as e:
                self.add_debug_message(f"Error receiving data: {e}")
                buffer = bytearray()
                state = "IDLE"
                time.sleep(0.1)
            
            # Check connection health based on time since last update or ack
            if time.time() - self.last_update_time > 10.0:
                if self.connection_healthy:
                    self.add_debug_message("Connection timed out (no data received for 10s)")
                self.connection_healthy = False
    
    def run_terminal_ui(self, stdscr):
        """Run the curses-based terminal UI"""
        # Setup colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)  # Good status
        curses.init_pair(2, curses.COLOR_RED, -1)    # Error/Warning
        curses.init_pair(3, curses.COLOR_YELLOW, -1) # Autonomous mode
        curses.init_pair(4, curses.COLOR_BLUE, -1)   # Neutral info
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)# Commands
        curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_RED) # Emergency
        curses.init_pair(7, curses.COLOR_BLACK, curses.COLOR_WHITE) # Input box
        
        # Show cursor for input
        curses.curs_set(1)
        
        # Enable keypad mode
        stdscr.keypad(True)
        
        # Don't wait for input when getch is called
        stdscr.nodelay(True)
        
        # Send initial status request
        self.send_command(9)
        
        while self.running:
            try:
                # Get terminal size
                max_y, max_x = stdscr.getmaxyx()
                
                # Clear the screen
                stdscr.clear()
                
                # Draw header
                header = " SAILBOT REMOTE CONTROL "
                stdscr.addstr(0, (max_x - len(header)) // 2, header, curses.A_BOLD)
                
                # Draw connection status
                if self.connection_healthy:
                    conn_status = "CONNECTED"
                    conn_color = curses.color_pair(1)
                else:
                    conn_status = "DISCONNECTED"
                    conn_color = curses.color_pair(2)
                    
                stdscr.addstr(0, max_x - len(conn_status) - 2, conn_status, conn_color | curses.A_BOLD)
                
                # Draw horizontal line
                stdscr.addstr(1, 0, "=" * max_x)
                
                # Draw command menu
                stdscr.addstr(2, 2, "COMMANDS:", curses.A_BOLD)
                line = 3
                
                for key, cmd in self.commands.items():
                    # Skip emergency stop for special formatting
                    if cmd["code"] != 4:
                        cmd_str = f"{key}: {cmd['name']} - {cmd['description']}"
                        if len(cmd_str) > max_x - 4:
                            cmd_str = cmd_str[:max_x - 7] + "..."
                        stdscr.addstr(line, 4, cmd_str, curses.color_pair(5))
                        line += 1
                
                # Draw emergency stop with special formatting
                stdscr.addstr(line, 4, "4: EMERGENCY STOP", curses.color_pair(6) | curses.A_BOLD)
                line += 1
                
                # Draw another horizontal line
                line += 1
                stdscr.addstr(line, 0, "-" * max_x)
                line += 1
                
                # Draw status information
                stdscr.addstr(line, 2, "BOAT STATUS:", curses.A_BOLD)
                line += 1
                
                # Status area
                if self.connection_healthy:
                    stdscr.addstr(line, 4, "Connection established with sailbot")
                    line += 1
                    
                    # Last update time
                    time_diff = time.time() - self.last_update_time
                    update_str = f"Last Update: {time_diff:.1f}s ago"
                    update_color = curses.color_pair(1) if time_diff < 5 else \
                                  curses.color_pair(3) if time_diff < 10 else \
                                  curses.color_pair(2)
                    stdscr.addstr(line, 4, update_str, update_color)
                    line += 1
                else:
                    stdscr.addstr(line, 4, "No connection with sailbot", curses.color_pair(2))
                    line += 1
                
                # Draw debug console (show last few messages)
                line += 1
                stdscr.addstr(line, 0, "-" * max_x)
                line += 1
                stdscr.addstr(line, 2, "DEBUG CONSOLE:", curses.A_BOLD)
                line += 1
                
                # Show last 10 debug messages (or less if not enough space)
                remaining_lines = max_y - line - 3  # -3 for input area and footer
                num_messages = min(remaining_lines, 10, len(self.debug_messages))
                
                for i in range(num_messages):
                    msg_idx = len(self.debug_messages) - num_messages + i
                    if 0 <= msg_idx < len(self.debug_messages):
                        msg = self.debug_messages[msg_idx]
                        if len(msg) > max_x - 4:
                            msg = msg[:max_x - 7] + "..."
                        stdscr.addstr(line, 4, msg)
                        line += 1
                
                # Draw input area
                line = max_y - 2
                stdscr.addstr(line, 0, "-" * max_x)
                line += 1
                stdscr.addstr(line, 2, "Command: ", curses.A_BOLD)
                
                # Draw input box
                input_box_start = 11
                input_box_width = max_x - input_box_start - 2
                stdscr.addstr(line, input_box_start, " " * input_box_width, curses.color_pair(7))
                
                # Show input buffer
                if len(self.input_buffer) > input_box_width:
                    visible_input = self.input_buffer[-(input_box_width):]
                else:
                    visible_input = self.input_buffer
                
                stdscr.addstr(line, input_box_start, visible_input, curses.color_pair(7))
                
                # Position cursor at end of input
                cursor_x = input_box_start + len(visible_input)
                if cursor_x >= max_x:
                    cursor_x = max_x - 1
                stdscr.move(line, cursor_x)
                
                # Refresh the screen
                stdscr.refresh()
                
                # Check for key presses
                key = stdscr.getch()
                
                if key != -1:  # -1 means no key pressed
                    if key == ord('q') and self.input_buffer == "":
                        # Only quit if input buffer is empty
                        self.running = False
                    elif key == curses.KEY_ENTER or key == 10 or key == 13:
                        # Process command
                        if self.input_buffer.strip() in self.commands:
                            command_code = self.commands[self.input_buffer.strip()]["code"]
                            self.send_command(command_code)
                        else:
                            self.add_debug_message(f"Unknown command: {self.input_buffer}")
                        self.input_buffer = ""
                    elif key == curses.KEY_BACKSPACE or key == 127 or key == 8:
                        # Handle backspace
                        if len(self.input_buffer) > 0:
                            self.input_buffer = self.input_buffer[:-1]
                    elif 32 <= key <= 126:  # Printable ASCII
                        # Add to input buffer
                        if len(self.input_buffer) < input_box_width:
                            self.input_buffer += chr(key)
                
                # Sleep to reduce CPU usage
                time.sleep(0.05)
                
            except Exception as e:
                # In case of error, close curses and print the error
                curses.endwin()
                print(f"Error in terminal UI: {e}")
                self.running = False
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)
        if self.serial_conn:
            self.serial_conn.close()

def main():
    parser = argparse.ArgumentParser(description='Sailbot Remote Control Terminal')
    parser.add_argument('--port', default='/dev/tty.usbserial-ABSCDPWR',
                        help='Serial port for RFD900x radio')
    parser.add_argument('--baud', type=int, default=57600,
                        help='Baud rate for serial connection')
    args = parser.parse_args()
    
    print("Initializing Sailbot Remote Control...")
    print(f"Connecting to radio on {args.port} at {args.baud} baud...")
    
    remote = SailbotRemoteControl(args.port, args.baud)
    
    try:
        # Run the terminal UI using curses
        curses.wrapper(remote.run_terminal_ui)
    except KeyboardInterrupt:
        pass
    finally:
        remote.cleanup()
        print("Sailbot Remote Control terminated.")

if __name__ == "__main__":
    main()