import Jetson.GPIO as GPIO
import time

# Pin definition (BOARD numbering)
PWM_PIN = 33  # Change to the pin you are using

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Set PWM frequency to 50Hz (standard for hobby servos)
pwm = GPIO.PWM(PWM_PIN, 50)  
pwm.start(0)  # Start PWM with 0% duty cycle

def set_servo_angle(angle):
    """
    Converts an angle (0-180) to a duty cycle (2.5-12.5%)
    0 degrees -> 2.5% duty cycle
    180 degrees -> 12.5% duty cycle
    """
    duty_cycle = 2.5 + (angle / 180.0) * 10.0
    pwm.ChangeDutyCycle(duty_cycle)

try:
    while True:
        # Sweep from 0 to 180 degrees
        for angle in range(0, 181, 5):
            set_servo_angle(angle)
            time.sleep(0.05)
        # Sweep back from 180 to 0 degrees
        for angle in range(180, -1, -5):
            set_servo_angle(angle)
            time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting gracefully...")

finally:
    pwm.stop()
    GPIO.cleanup()

