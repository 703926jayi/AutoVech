#Controlled through Potentionmeter
import Jetson.GPIO as GPIO
import spidev
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BrakesNode(Node):
    def __init__(self, dir_pin1=33, dir_pin2=35, enable_pin=31, adc_channel=0, timer_period = 0.02):
        # establish the node name in the larger Ros2 system
        super().__init__("brakes_node")
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Motor control setup
        self.dir1 = dir_pin1
        self.dir2 = dir_pin2
        self.enable = enable_pin
        self.adc_channel = adc_channel

        GPIO.setup(self.dir1, GPIO.OUT)
        GPIO.setup(self.dir2, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)

        # PWM setup
        self.pwm = GPIO.PWM(self.enable, 1000)
        self.pwm.start(0)

        # SPI setup for ADC (MCP3008)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000

        self.target_position = 0.0

        # Control parameters
        self.deadband = 0.02       
        self.kp = 100.0  

        # Subscribe to brake position topic
        self.subscription = self.create_subscription(
            Float32,
            'brake_position',
            self.brake_callback,
            10
        )

    def read_adc(self):
        """Read analog input from MCP3008."""
        r = self.spi.xfer2([1, (8 + self.adc_channel) << 4, 0])
        return ((r[1] & 3) << 8) + r[2]
    
    def brake_callback(self, msg: Float32, controller_connected):
        """Update desired brake position."""
        if controller_connected == True:
            self.target_position = max(0.0, min(1.0, msg.data))
        else:
            self.target_position = 1.0

    def control_loop(self):
        """Compare potentiometer position to target and move actuator."""
        adc_value = self.read_adc()
        actual_position = adc_value / 1023.0

        error = self.target_position - actual_position

        if abs(error) < self.deadband:
            self.stop()
            return

        speed_pct = min(100.0, abs(error) * self.kp)

        if error > 0:
            self.move('extend', speed_pct)
        else:
            self.move('retract', speed_pct)


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

def brakes_start(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = BrakesNode(timer_period=0.02)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # shutdown the ROS2 communication
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()
