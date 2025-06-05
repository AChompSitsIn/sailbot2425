#!/usr/bin/env python3
import sys
from smbus2 import SMBus

I2C_BUS = 1
ARDUINO_ADDR = 0x09
SERVO_CMD = 0x20
NEUTRAL_SERVO_ANGLE = 57  # True 0 position for rudder
MIN_RUDDER_ANGLE = -21    # Maximum left
MAX_RUDDER_ANGLE = 21     # Maximum right

def send_rudder_angle(rudder_angle):
    """
    Send rudder angle to Arduino via I2C.
    
    Args:
        rudder_angle: Rudder angle in degrees (-21 to +21)
                     Negative = left, Positive = right, 0 = neutral
    """
    # Constrain rudder angle to safe range
    rudder_angle = max(MIN_RUDDER_ANGLE, min(MAX_RUDDER_ANGLE, float(rudder_angle)))
    
    # Convert rudder angle to servo angle
    # Neutral (0°) = 57° servo, add rudder angle to move left/right
    servo_angle = int(NEUTRAL_SERVO_ANGLE + rudder_angle)
    
    # Send command to Arduino
    with SMBus(I2C_BUS) as bus:
        bus.write_i2c_block_data(ARDUINO_ADDR, SERVO_CMD, [servo_angle])
        print(f"Rudder angle: {rudder_angle:+.1f}° → Servo angle: {servo_angle}° → Arduino at 0x{ARDUINO_ADDR:02X}")

def initialize_rudder():
    """Set rudder to neutral position (0°)"""
    send_rudder_angle(0)
    print("Rudder initialized to neutral position")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # No arguments - initialize to neutral
        initialize_rudder()
    elif len(sys.argv) == 2:
        try:
            angle = float(sys.argv[1])
            send_rudder_angle(angle)
        except ValueError:
            print("Error: Angle must be a number")
            sys.exit(1)
    else:
        print("Usage: sudo python3 i2c_rudder_control.py [angle]")
        print("  angle: Rudder angle in degrees (-21 to +21)")
        print("         Negative = left, Positive = right")
        print("  No angle: Initialize to neutral (0°)")
        sys.exit(1)
