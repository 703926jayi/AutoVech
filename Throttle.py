#!/usr/bin/env python3
import time
import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ThrottleNode(Node):

    FREQ = 333
    PULSE_NEUTRAL = 1875    # µs — neutral/min (was VAL_MIN, renamed for clarity)
    PULSE_MAX     = 1306   # µs — full throttle (was VAL_MAX, renamed for clarity)

    def __init__(self, pin=12):

        super().__init__("throttle_node")

        self.pin = pin
        self.current_throttle = 0.0
        self.controller_connected = False

        # GPIO setup
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)

        # CHANGED: arm the ESC by holding neutral for 3 seconds before accepting commands
        self.get_logger().info("Arming ESC — holding neutral...")
        neutral_duty = self.to_duty(self.PULSE_NEUTRAL)
        lgpio.tx_pwm(self.h, self.pin, self.FREQ, neutral_duty)
        time.sleep(3)
        self.get_logger().info("ESC armed")

        # Throttle subscriber
        self.angle_sub = self.create_subscription(
            Float32,
            'throttle_angle',
            self.angle_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Connection subscriber
        self.connection_sub = self.create_subscription(
            Bool,
            'is_connected',
            self.connection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def to_duty(self, pulse_us):
        period_us = (1 / self.FREQ) * 1_000_000
        return (pulse_us / period_us) * 100

    def angle_callback(self, msg: Float32):
        self.current_throttle = max(0.0, min(1.0, msg.data))
        self.update_throttle()

    def connection_callback(self, msg: Bool):
        self.controller_connected = msg.data
        self.update_throttle()

    def update_throttle(self):
        if self.controller_connected:
            # CHANGED: was VAL_MIN - (VAL_MIN - VAL_MAX) * throttle (confusing inverted naming)
            # Now clearly: neutral + (max - neutral) * throttle
            pulse = self.PULSE_NEUTRAL + (self.PULSE_MAX - self.PULSE_NEUTRAL) * self.current_throttle
            duty = self.to_duty(pulse)
        else:
            duty = self.to_duty(self.PULSE_NEUTRAL)

        lgpio.tx_pwm(self.h, self.pin, self.FREQ, duty)

    def cleanup(self):
        lgpio.tx_pwm(self.h, self.pin, 0, 0)
        lgpio.gpiochip_close(self.h)


def start_throttle(args=None):

    rclpy.init(args=args)

    node = ThrottleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()
