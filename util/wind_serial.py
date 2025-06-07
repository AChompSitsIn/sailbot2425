#!/usr/bin/env python3
"""
Simple wind sensor reader for Raspberry Pi
Reads wind data from Arduino over USB serial
"""

import serial
import time
import sys

class WindSensorReader:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200):
        """
        Initialize wind sensor reader
        
        Args:
            port: Serial port (usually /dev/ttyACM0 or /dev/ttyUSB0)
            baudrate: Baud rate (must match Arduino)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.current_wind_direction = 0.0
        
    def connect(self):
        """Connect to Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            # Clear any startup messages
            time.sleep(2)
            self.serial_conn.reset_input_buffer()
            print(f"Connected to wind sensor on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False
    
    def read_wind_data(self):
        """
        Read and parse wind data from Arduino
        
        Returns:
            float: Wind direction in degrees, or None if error
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
            
        try:
            if self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                # Parse the data
                if line.startswith('WIND,'):
                    parts = line.split(',')
                    if len(parts) == 2:
                        wind_degrees = float(parts[1])
                        self.current_wind_direction = wind_degrees
                        return wind_degrees
                elif line.startswith('ERROR,'):
                    print(f"Sensor error: {line}")
                    return None
                    
        except Exception as e:
            print(f"Error reading data: {e}")
            
        return None
    
    def get_current_wind(self):
        """Get the last known wind direction"""
        return self.current_wind_direction
    
    def close(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Connection closed")

def main():
    """Example usage"""
    # Use only ACM0
    reader = WindSensorReader(port='/dev/ttyACM1')
    
    if not reader.connect():
        print("Failed to connect to /dev/ttyACM0")
        sys.exit(1)
    
    print("Reading wind data... (Ctrl+C to stop)")
    
    try:
        while True:
            wind_dir = reader.read_wind_data()
            if wind_dir is not None:
                print(f"Wind direction: {wind_dir:.1f}°")
            time.sleep(0.1)  # Small delay
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        reader.close()

if __name__ == "__main__":
    main()
