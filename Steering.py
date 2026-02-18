"""
Minimal example for controlling an ODrive via the CANSimple protocol.
 
Puts the ODrive into closed loop control mode, sends periodic sinusoidal velocity
setpoints, and asynchronously prints the encoder feedback.
 
Assumes that the ODrive is already configured for velocity control.
 
If the watchdog is enabled on the ODrive, it is fed implicitly by the continuous
velocity setpoint message and the motor will stop when the script is terminated.
The heartbeat interval should be shorter than the watchdog timeout to ensure
timely confirmation of the axis entering closed loop control mode without
triggering the watchdog.
 
See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""
 
import can
import struct
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SteeringNode(Node):

    def __init__(self):
        super().__init__("steering_node")

        # ================= CONFIG =================

        self.node_id = 1

        self.MAX_POSITION = 0.4
        self.MAX_RATE = 1.5            # max change per second (units/sec)
        self.CONTROL_PERIOD = 0.02     # 50 Hz
        self.HEARTBEAT_TIMEOUT = 0.5
        self.COMMAND_TIMEOUT = 0.25
        self.RECOVERY_COOLDOWN = 1.0

        # ==========================================

        # CAN
        self.bus = can.interface.Bus("can0", interface="socketcan")

        # State
        self.desired_position = 0.0
        self.filtered_position = 0.0

        self.axis_state = 0
        self.axis_error = 0
        self.in_closed_loop = False

        self.last_heartbeat_time = time.time()
        self.last_command_time = time.time()
        self.last_recovery_attempt = 0.0

        # ROS subscriber
        self.create_subscription(
            Float32,
            "steering_position",
            self.position_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Timers
        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD,
            self.control_loop
        )

        self.heartbeat_timer = self.create_timer(
            0.02,
            self.poll_heartbeat
        )

        self.get_logger().info("Steering node started")

    # =====================================================
    # ROS CALLBACK
    # =====================================================

    def position_callback(self, msg: Float32):

        # Saturate input
        value = max(-1.0, min(1.0, msg.data))

        self.desired_position = value
        self.last_command_time = time.time()

    # =====================================================
    # MAIN CONTROL LOOP (50 Hz deterministic)
    # =====================================================

    def control_loop(self):

        now = time.time()

        # -------- Command Timeout Fail-Safe --------
        if now - self.last_command_time > self.COMMAND_TIMEOUT:
            self.desired_position = 0.0

        # -------- Heartbeat Timeout --------
        if now - self.last_heartbeat_time > self.HEARTBEAT_TIMEOUT:
            if self.in_closed_loop:
                self.get_logger().error("Heartbeat timeout — dropping closed loop")
            self.in_closed_loop = False

        # -------- Error Handling --------
        if self.axis_error != 0:
            self.get_logger().error(f"ODrive axis error: {self.axis_error}")
            self.in_closed_loop = False
            return

        # -------- Closed Loop Recovery --------
        if not self.in_closed_loop:
            if now - self.last_recovery_attempt > self.RECOVERY_COOLDOWN:
                self.request_closed_loop()
                self.last_recovery_attempt = now
            return

        # -------- Rate Limiting --------
        max_step = self.MAX_RATE * self.CONTROL_PERIOD
        delta = self.desired_position - self.filtered_position

        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)

        self.filtered_position += delta

        # -------- Convert to ODrive Position --------
        pos_setpoint = self.MAX_POSITION * self.filtered_position

        # -------- Send Command --------
        try:
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x0C),
                data=struct.pack('<fhh', pos_setpoint, 0, 0),
                is_extended_id=False
            ))
        except can.CanError:
            self.get_logger().error("CAN transmission failure")

    # =====================================================
    # REQUEST CLOSED LOOP
    # =====================================================

    def request_closed_loop(self):
        try:
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x07),
                data=struct.pack('<I', 8),
                is_extended_id=False
            ))
            self.get_logger().warn("Requesting closed loop control")
        except can.CanError:
            self.get_logger().error("Failed to send closed loop request")

    # =====================================================
    # HEARTBEAT POLLING (Non-blocking)
    # =====================================================

    def poll_heartbeat(self):

        msg = self.bus.recv(timeout=0.0)
        if msg is None:
            return

        if msg.arbitration_id != (self.node_id << 5 | 0x01):
            return

        self.last_heartbeat_time = time.time()

        error, state, result, traj_done = struct.unpack(
            '<IBBB',
            bytes(msg.data[:7])
        )

        self.axis_error = error
        self.axis_state = state

        if error != 0:
            return

        if state == 8:
            if not self.in_closed_loop:
                self.get_logger().info("Closed loop engaged")
            self.in_closed_loop = True
        else:
            self.in_closed_loop = False

    # =====================================================
    # CLEANUP
    # =====================================================

    def cleanup(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass
        self.get_logger().info("Steering node shutdown cleanly")


# =========================================================
# MAIN
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
