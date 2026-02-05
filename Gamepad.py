#!/usr/bin/env python3

from inputs import get_gamepad
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile
import threading


class GamepadNode(Node):

    def __init__(self, timer_period=0.02):
        self.left_stick_y = 0.0
        self.right_trigger = 0.0
        self.left_trigger = 0.0
        self.button_a = False
        self.button_b = False
        self.connected =False

        super().__init__('gamepad_node')
        
        #Safety Topic Creation
        self.publisher_connected = self.create_publisher(
            Bool,
            'is_connected',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        #Throttle Topic Creation
        self.publisher_throttle = self.create_publisher(
            Float32,
            'throttle_angle',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        #Brakes Topic Creation
        self.publisher_brakes = self.create_publisher(
            Float32,
            'brake_position',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        #Topic Callbacker
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Allow controller inputting to operate in seperate thread so no hangups
        self._running = True
        self.gamepad_thread = threading.Thread(
            target=self.gamepad_loop,
            daemon=True
        )
        self.gamepad_thread.start()

    def timer_callback(self):
        #Publishing for is_connected
        msg_connected = Bool()
        msg_connected.data = self.connected
        self.publisher_connected.publish(msg_connected)

        #Publishing for Throttle
        msg_throttle = Float32()
        msg_throttle.data = self.right_trigger
        self.publisher_throttle.publish(msg_throttle)

        #Publishing for Brakes
        msg_brakes = Float32()
        msg_brakes.data = self.left_trigger
        self.publisher_brakes.publish(msg_brakes)
    
    def gamepad_loop(self):
        while self._running:
            try:
                events = get_gamepad()
                self.connected = True
                for event in events:
                    if event.code == 'ABS_X':          # Left stick horizontal
                        self.left_stick_x = event.state / 32767.0
                    elif event.code == 'ABS_RZ':       # Right trigger
                        self.right_trigger = event.state / 255.0
                    elif event.code == 'ABS_Z':        # Left trigger
                        self.left_trigger = event.state / 255.0
                    elif event.code == 'BTN_SOUTH':    # A button
                        self.button_a = event.state == 1
                    elif event.code == 'BTN_EAST':     # B button
                        self.button_b = event.state == 1
            except Exception as e:
                self.connected = False
                self.get_logger().warn(f"Gamepad read error: {e}")

    

def gamepad_start(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = GamepadNode(timer_period=0.02)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # shutdown the ROS2 communication
    node.destroy_node()
    rclpy.shutdown()