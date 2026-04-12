# Controlled through Potentiometer
import lgpio
import spidev
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class BrakesNode(Node):
    def __init__(self, dir_pin1=33, dir_pin2=35, enable_pin=31, adc_channel=0):
        super().__init__("brakes_node")

        #GPIO (lgpio)
        self.h = lgpio.gpiochip_open(0)

        self.dir1 = dir_pin1
        self.dir2 = dir_pin2
        self.enable = enable_pin
        self.adc_channel = adc_channel

        lgpio.gpio_claim_output(self.h, self.dir1)
        lgpio.gpio_claim_output(self.h, self.dir2)
        lgpio.gpio_claim_output(self.h, self.enable)

        # PWM 
        self.pwm_freq = 1000
        self.pwm_duty = 0
        lgpio.tx_pwm(self.h, self.enable, self.pwm_freq, 0)

        #SPI ADC (MCP3008)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000

        #State
        self.target_position = 1.0
        self.controller_connected = False
        self.current_brake_pos = 0.0

        # Control parameters
        self.deadband = 0.02
        self.kp = 100.0

        # ROS
        self.create_timer(0.02, self.control_loop)

        self.brake_sub = self.create_subscription(
            Float32,
            'brake_position',
            self.brake_callback,
            10
        )

        self.connection_sub = self.create_subscription(
            Bool,
            'is_connected',
            self.connection_callback,
            10
        )

    # Callbacks

    def brake_callback(self, msg: Float32):
        self.current_brake_pos = msg.data
        self.brake_position()

    def connection_callback(self, msg: Bool):
        self.controller_connected = msg.data
        self.brake_position()

    # ADC
    
    def read_adc(self):
        r = self.spi.xfer2([1, (8 + self.adc_channel) << 4, 0])
        return ((r[1] & 3) << 8) + r[2]

    # Control logic

    def brake_position(self):
        if self.controller_connected:
            self.target_position = max(0.0, min(1.0, self.current_brake_pos))
        else:
            self.target_position = 1.0

    def control_loop(self):
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

    # ───────────────────────────────────────────────────────────────
    # Motor control (lgpio replacement)
    # ───────────────────────────────────────────────────────────────

    def move(self, direction, speed_pct=100):
        speed_pct = max(0.0, min(100.0, speed_pct))

        if direction == 'extend':
            lgpio.gpio_write(self.h, self.dir1, 1)
            lgpio.gpio_write(self.h, self.dir2, 0)

        elif direction == 'retract':
            lgpio.gpio_write(self.h, self.dir1, 0)
            lgpio.gpio_write(self.h, self.dir2, 1)

        else:
            self.stop()
            return

        lgpio.tx_pwm(self.h, self.enable, self.pwm_freq, speed_pct)

    def stop(self):
        lgpio.gpio_write(self.h, self.dir1, 0)
        lgpio.gpio_write(self.h, self.dir2, 0)
        lgpio.tx_pwm(self.h, self.enable, self.pwm_freq, 0)

    # Cleanup

    def cleanup(self):
        self.stop()
        lgpio.tx_pwm(self.h, self.enable, 0, 0)
        lgpio.gpiochip_close(self.h)
        self.spi.close()


def brakes_start(args=None):
    rclpy.init(args=args)
    node = BrakesNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()
