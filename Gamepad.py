#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile
import threading
import serial
import sys
import logging

log = logging.getLogger("rc")
logging.basicConfig(level=logging.INFO)


class GamepadNode(Node):

    SERIAL_PORT = '/dev/ttyAMA0'
    BAUD_RATE = 115200
    NUM_CHANNELS = 6
    FRAME_START = [0xFF, 0xAA]

    def __init__(self, timer_period=0.02, port=SERIAL_PORT, baudrate=BAUD_RATE):

        super().__init__('gamepad_node')

        self.left_stick_x = 0.0
        self.right_trigger = 0.0
        self.left_trigger = 0.0
        self.mode = 0
        self.connected = False
        self.timer_period = timer_period

        # Serial state
        self.ser = None
        self.success_count = 0
        self.error_count = 0
        self.port = port

        # Try serial ports
        ports_to_try = [port, '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/serial0']

        for try_port in ports_to_try:
            try:
                log.info(f"Trying serial {try_port}...")
                self.ser = serial.Serial(try_port, baudrate, timeout=1)

                time.sleep(2)
                self.ser.reset_input_buffer()

                log.info(f"Serial opened: {try_port}")
                self.port = try_port
                break

            except serial.SerialException:
                if try_port == ports_to_try[-1]:
                    log.error("Could not open any serial port")
                    sys.exit(1)

        # Publishers

        self.publisher_connected = self.create_publisher(
            Bool,
            'is_connected',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.publisher_throttle = self.create_publisher(
            Float32,
            'throttle_angle',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.publisher_brakes = self.create_publisher(
            Float32,
            'brake_position',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.publisher_steering = self.create_publisher(
            Float32,
            'steering_position',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Background thread
        self._running = True

        self.gamepad_thread = threading.Thread(
            target=self.gamepad_loop,
            daemon=True
        )

        self.gamepad_thread.start()

    # ──────────────────────────────────────────
    # Frame Sync
    # ──────────────────────────────────────────

    def find_frame_start(self):

        while True:

            b = self.ser.read(1)

            if not b:
                return False

            if b[0] == self.FRAME_START[0]:

                b2 = self.ser.read(1)

                if b2 and b2[0] == self.FRAME_START[1]:
                    return True

    # ──────────────────────────────────────────
    # Channel Reader
    # ──────────────────────────────────────────

    def read_channels(self):

        try:

            if not self.find_frame_start():
                return None

            data = self.ser.read(self.NUM_CHANNELS)

            if len(data) != self.NUM_CHANNELS:
                return None

            self.success_count += 1

            return [b / 255.0 for b in data]

        except Exception as e:

            self.error_count += 1

            if self.error_count % 10 == 0:
                log.warning(f"Serial error: {e}")

            return None

    # ──────────────────────────────────────────
    # ROS Publisher Timer
    # ──────────────────────────────────────────

    def timer_callback(self):

        msg_connected = Bool()
        msg_connected.data = self.connected
        self.publisher_connected.publish(msg_connected)

        msg_throttle = Float32()
        msg_throttle.data = self.right_trigger
        self.publisher_throttle.publish(msg_throttle)

        msg_brakes = Float32()
        msg_brakes.data = self.left_trigger
        self.publisher_brakes.publish(msg_brakes)

        msg_steering = Float32()
        msg_steering.data = self.left_stick_x
        self.publisher_steering.publish(msg_steering)

    # ──────────────────────────────────────────
    # Serial Reader Thread
    # ──────────────────────────────────────────

    def gamepad_loop(self):

        while self._running:

            channels = self.read_channels()

            if not channels:
                self.connected = False
                continue

            self.connected = True

            ch1 = channels[0]
            ch2 = channels[1]

            # Map channels
            self.left_stick_x = ch1
            self.right_trigger = max(0.0, ch2 - 0.5) * 2
            self.left_trigger = max(0.0, 0.5 - ch2) * 2

    def destroy_node(self):

        self._running = False

        if self.ser:
            self.ser.close()

        super().destroy_node()


def gamepad_start(args=None):

    rclpy.init(args=args)

    node = GamepadNode(timer_period=0.02)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()