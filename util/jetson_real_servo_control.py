import Jetson.GPIO as GPIO
import time

# BOARD-mode pin 33 → the white PWM lead of your servo
servo_pin = 33

# Pulse widths in microseconds, matching your Arduino “safe” range
safe_min = 820    # µs
safe_max = 2280   # µs
neutral  = 1500  # µs

# Servo pulse period
period_us = 20000  # 20 ms → 50 Hz

# Compute duty % from pulse width
def duty_from_us(pulse_us):
    return (pulse_us / period_us) * 100.0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz
pwm.start(0)

def set_pulse(pulse_us, hold_s=0.2):
    dc = duty_from_us(pulse_us)
    pwm.ChangeDutyCycle(dc)
    time.sleep(hold_s)
    pwm.ChangeDutyCycle(0)  # stop further pulses

try:
    print("→ moving to neutral")
    set_pulse(neutral, hold_s=0.2)

    while True:
        print("→ moving to min")
        set_pulse(safe_min, hold_s=0.2)
        
        print("→ moving to max")
        set_pulse(safe_max, hold_s=0.2)
        
        print("→ back to neutral")
        set_pulse(neutral, hold_s=0.2)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
    print("cleaned up GPIO")
