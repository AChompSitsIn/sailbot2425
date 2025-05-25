#!/usr/bin/env python3
import serial
import time

# Connect to serial port
ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)
print("Connected to serial port")

# Send initial message
ser.write(bytearray([0xB0, 0x55, 0xAA]))
print("Sent initial message")

# Send status messages every 2 seconds
while True:
    # Check for incoming data
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print(f"RECEIVED: {data}")
    
    # Send GPS data
    gps_msg = "GPS,37.425041,-122.104624,1.23,2\n"
    ser.write(gps_msg.encode())
    print(f"Sent: {gps_msg.strip()}")
    
    # Send status
    status_msg = "STATUS,rc,developer_mode\n"
    ser.write(status_msg.encode())
    print(f"Sent: {status_msg.strip()}")
    
    time.sleep(2)
