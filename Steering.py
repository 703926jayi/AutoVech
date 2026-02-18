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
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile
 
class SteeringNode(Node):
    def __init__(self, timer_period = 0.02):
        super().__init__("steering_node")
        self.node_id = 1 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.
        self.bus = can.interface.Bus("can0", interface="socketcan")
        self.canbus_Loop()
        # Throttle angle subscriber
        self.position_sub = self.create_subscription(
            Float32,
            'steering_position',
            self.position_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
    def canbus_Loop(self):
        while rclpy.ok():
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue

            if msg.arbitration_id == (self.node_id << 5 | 0x01):
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == 8:
                    break
                
    def position_callback(self, ros_msg: Float32):
    
        # Put axis into closed loop control state
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
        ))
 
        # Wait for axis to enter closed loop control by scanning heartbeat messages
        

        max_pos = 0.4
        
        # Control ODrive while notifier object exist
        #with can.Notifier(bus, [on_rx_message]):
        
        pos_setpoint = max_pos - ros_msg.data * max_pos
            # Update velocity and reset watchdog timer
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0c), # 0x0c: Set_Input_Vel
            data=struct.pack('<fhh', pos_setpoint, 1, 1), # 0.0: torque feedforward
            is_extended_id=False
        ))
    
    def cleanup(self):
        self.bus.shutdown()

def throttle_start(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = SteeringNode(timer_period=0.02)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # shutdown the ROS2 communication
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()

    """# Handler for incoming CAN messages to print encoder feedback
        def on_rx_message(msg: can.Message):
            if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
                pos, vel = struct.unpack('<ff', bytes(msg.data))
                print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")"""