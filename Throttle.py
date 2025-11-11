import Jetson.GPIO as GPIO
import time

class Throttle:
    def __init__(self, pin=33, freq=50):
        self.pin = pin
        self.freq = freq
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)

    def set_angle(self, angle):
        """Set servo angle between 0 and 180 degrees."""
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.pwm.stop()
