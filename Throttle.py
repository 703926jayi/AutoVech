#!/usr/bin/env python3

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ThrottleNode(Node):
    def __init__(self, pin=33, timer_period=0.02):
        super().__init__("throttle_node")

        self.pin = pin
        self.freq = 50 

        #lgpio setup 
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)

        # initial state
        self.current_angle = 120
        self.controller_connected = False

        initial_duty = self.angle_to_duty(self.current_angle)
        lgpio.tx_pwm(self.h, self.pin, self.freq, initial_duty)

        # Subscribers
        self.angle_sub = self.create_subscription(
            Float32,
            'throttle_angle',
            self.angle_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.connection_sub = self.create_subscription(
            Bool,
            'is_connected',
            self.connection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    #Callbacks 

    def angle_callback(self, msg: Float32):
        self.current_angle = 120 - 65 * msg.data
        self.update_servo()

    def connection_callback(self, msg: Bool):
        self.controller_connected = msg.data
        self.update_servo()


    def update_servo(self):
        if self.controller_connected:
            angle = max(55, min(120, self.current_angle))
        else:
            angle = 180

        self.set_angle(angle)

    def set_angle(self, angle):
        duty_cycle = self.angle_to_duty(angle)
        lgpio.tx_pwm(self.h, self.pin, self.freq, duty_cycle)

    def angle_to_duty(self, angle):
        return 2.5 + (angle / 180.0) * 10.0

    def cleanup(self):
        lgpio.tx_pwm(self.h, self.pin, 0, 0)
        lgpio.gpiochip_close(self.h)


def start_throttle(args=None):
    rclpy.init(args=args)

    node = ThrottleNode(pin=33, timer_period=0.02)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()
