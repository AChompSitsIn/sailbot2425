# Jetson Servo Control using PWM5 (pin 33)
import Jetson.GPIO as GPIO
import time

# Use physical pin 33 which is PWM5 according to the header tool
servo_pin = 33

# Servo constants
SERVO_MIN_DUTY = 2.5  # Duty cycle for 0 degrees
SERVO_MAX_DUTY = 12.5  # Duty cycle for 180 degrees
SERVO_FREQ = 50  # PWM frequency in Hz (20ms period)

def angle_to_duty_cycle(angle):
    """Convert servo angle (0-180) to duty cycle (2.5-12.5%)"""
    return SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)

try:
    # Initialize GPIO
    print(f"Initializing GPIO on pin {servo_pin} (BOARD numbering)")
    GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
    GPIO.setup(servo_pin, GPIO.OUT)
    
    # Initialize PWM
    pwm = GPIO.PWM(servo_pin, SERVO_FREQ)
    
    # Start PWM with 0 degrees position
    pwm.start(angle_to_duty_cycle(0))
    print(f"Servo initialized at 0 degrees on pin {servo_pin}")
    time.sleep(1)  # Give it time to move to starting position
    
    # Now do continuous sweep
    print("Starting continuous sweep...")
    while True:
        # Sweep from 0 to 180 degrees
        print("Moving from 0 to 180 degrees")
        for angle in range(0, 181, 5):  # Increment by 5 degrees for smoother motion
            duty_cycle = angle_to_duty_cycle(angle)
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)  # Slightly longer delay for smoother motion
            
        time.sleep(0.5)  # Pause at 180 degrees
        
        # Sweep from 180 to 0 degrees
        print("Moving from 180 to 0 degrees")
        for angle in range(180, -1, -5):  # Decrement by 5 degrees
            duty_cycle = angle_to_duty_cycle(angle)
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
            
        time.sleep(0.5)  # Pause at 0 degrees

except ValueError as e:
    print(f"Error: {e}")
    print("Make sure PWM5 is enabled on pin 33 using the Jetson Expansion Header Tool")
    
except KeyboardInterrupt:
    print("Program stopped by user")

except Exception as e:
    print(f"An error occurred: {e}")
    import traceback
    traceback.print_exc()

finally:
    # Clean up resources
    try:
        if 'pwm' in locals():
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")
    except Exception as e:
        print(f"Error during cleanup: {e}")