#!/usr/bin/env python3
import smbus
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile
import threading


class GamepadNode(Node):

    def __init__(self, timer_period=0.02):
        self.left_stick_x = 0.0
        self.right_trigger = 0.0
        self.left_trigger = 0.0
        self.mode = 0
        self.connected =False
        self.timer_period = timer_period

        #I2C Inits
        self.BUS_NUMBER = 1        # Change if needed (check with i2cdetect)
        self.ADDRESS = 0x08
        self.NUM_CHANNELS = 6

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
        #Steering Topic Creation
        self.publisher_steering = self.create_publisher(
            Float32,
            'steering_position',
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

        #Publishing for Steering
        msg_steering = Float32()
        msg_steering.data = self.left_stick_x
        self.publisher_brakes.publish(msg_steering)
    
    def gamepad_loop(self):
       
        bus = smbus.SMBus(self.BUS_NUMBER)
       
        while True:
            try:
                self.connected = True
                self.get_logger().info('Controller Connected')
                # Read exactly 6 bytes
                data = bus.read_i2c_block_data(self.ADDRESS, 0, self.NUM_CHANNELS)
                
                # Convert back to normalized float (0.0–1.0)
                channels = [byte / 255.0 for byte in data]

                for i, value in enumerate(channels):

                    self.get_logger().info(f"CH{i+1}: {value:.3f}", end="  ")

                    if i == 1:
                        if value > 0.5:
                            self.right_trigger = ((value-0.5)*2)
                            self.left_trigger = 0
                        else:
                            self.right_trigger = 0
                            self.left_trigger = ((abs(value)-0.5)*2)
                    if i == 0:
                        self.left_stick_x = ((value-0.5)*2)

                print()
                time.sleep(self.timer_period)

            except Exception as e:
                self.connected = False
                print("I2C Error:", e)
                time.sleep(self.timer_period)
            
    

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
    """
    Notes run sudo apt install python3-smbus for this to work.
    Use these as checks for bus number
    sudo apt install i2c-tools
    sudo i2cdetect -y 1
    """