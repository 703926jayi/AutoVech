#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ThrottleNode(Node):
    def __init__(self, pin=33, timer_period=0.02):
        # establish the node name in the larger Ros2 system
        super().__init__("throttle_node")
        self.pin = pin
        self.freq = 1 / timer_period
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)

       # Throttle angle subscriber
        self.angle_sub = self.create_subscription(
            Float32,
            'throttle_angle',
            self.angle_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Controller connection subscriber
        self.connection_sub = self.create_subscription(
            Bool,
            'is_connected',
            self.connection_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def angle_callback(self, msg: Float32):
        self.current_angle = msg.data
        self.update_servo()

    def connection_callback(self, msg: Bool):
        self.controller_connected = msg.data
        self.update_servo()

    def update_servo(self):
        if self.controller_connected:
            angle = max(0, min(180, self.current_angle))
        else:
            angle = 0
        self.set_angle(angle)

    def set_angle(self, angle):
        """Set servo angle between 0 and 180 degrees."""
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

def throttle_start(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = ThrottleNode(timer_period=0.02)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # shutdown the ROS2 communication
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()