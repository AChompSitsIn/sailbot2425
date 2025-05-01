import smbus
import time

# Define I2C bus
bus = smbus.SMBus(1)  # Use I2C bus 1

# Arduino addresses
RUDDER_ADDRESS = 0x09
WINCH_ADDRESS = 0x08  # Not used in this script but defined for reference

# Rudder position limits
MIN_POSITION = 30
MAX_POSITION = 160

def set_rudder_position(position):
    """Set the rudder position (angle).
    
    Args:
        position: Angle between MIN_POSITION and MAX_POSITION
    """
    # Ensure position is within limits
    position = max(MIN_POSITION, min(MAX_POSITION, position))
    
    try:
        # Send position to Arduino
        bus.write_byte(RUDDER_ADDRESS, position)
        print(f"Sent rudder position: {position}")
    except Exception as e:
        print(f"Error sending rudder position: {e}")

def test_rudder():
    """Test the rudder by moving it between min, max, and center positions."""
    print("Testing rudder movement...")
    
    # Move to center
    print("Moving to center position (90 degrees)")
    set_rudder_position(90)
    time.sleep(2)
    
    # Move to minimum position
    print(f"Moving to minimum position ({MIN_POSITION} degrees)")
    set_rudder_position(MIN_POSITION)
    time.sleep(2)
    
    # Move to maximum position
    print(f"Moving to maximum position ({MAX_POSITION} degrees)")
    set_rudder_position(MAX_POSITION)
    time.sleep(2)
    
    # Return to center
    print("Returning to center position (90 degrees)")
    set_rudder_position(90)

if __name__ == "__main__":
    try:
        test_rudder()
        
        # Interactive control
        print("\nEnter rudder positions (30-160) or 'q' to quit:")
        while True:
            cmd = input("> ")
            if cmd.lower() == 'q':
                break
            try:
                position = int(cmd)
                set_rudder_position(position)
            except ValueError:
                print("Please enter a number between 30 and 160")
                
    except KeyboardInterrupt:
        print("\nExiting rudder control program")