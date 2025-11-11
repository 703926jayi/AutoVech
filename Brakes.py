#Controlled through Potentionmeter
import Jetson.GPIO as GPIO
import spidev
import time

class Brakes:
    def __init__(self, dir_pin1=33, dir_pin2=35, enable_pin=31, pwm_freq=1000, adc_channel=0):
        # Motor control setup
        self.dir1 = dir_pin1
        self.dir2 = dir_pin2
        self.enable = enable_pin
        self.adc_channel = adc_channel

        GPIO.setup(self.dir1, GPIO.OUT)
        GPIO.setup(self.dir2, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)

        # PWM setup
        self.pwm = GPIO.PWM(self.enable, pwm_freq)
        self.pwm.start(0)

        # SPI setup for ADC (MCP3008)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000

    def read_adc(self):
        """Read analog input from MCP3008."""
        r = self.spi.xfer2([1, (8 + self.adc_channel) << 4, 0])
        return ((r[1] & 3) << 8) + r[2]

    def move(self, direction, speed_pct=100):
        """Move actuator in 'extend' or 'retract' direction at given speed."""
        if direction == 'extend':
            GPIO.output(self.dir1, GPIO.HIGH)
            GPIO.output(self.dir2, GPIO.LOW)
        elif direction == 'retract':
            GPIO.output(self.dir1, GPIO.LOW)
            GPIO.output(self.dir2, GPIO.HIGH)
        else:
            self.stop()
            return

        self.pwm.ChangeDutyCycle(max(0, min(100, speed_pct)))

    def stop(self):
        GPIO.output(self.dir1, GPIO.LOW)
        GPIO.output(self.dir2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()
        self.pwm.stop()
        self.spi.close()
