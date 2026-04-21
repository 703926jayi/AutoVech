# Architecture Overview:
[RC Controller] -> (RF) -> [Receiver] -> (PWM/Serial) -> [Arduino] -> (Serial over /dev/ttyAMA0) -> [GamepadNode (ROS2)]

The GamepadNode acts as the central dispatcher. It reads the serial frame, splits it into distinct topics, and publishes them:
- /throttle_angle   -> ThrottleNode (Controls Servo via lgpio PWM)
- /brake_position   -> BrakesNode (Controls linear actuator with ADC feedback)
- /steering_position -> SteeringNode (Controls ODrive motor via CAN bus)
- /is_connected     -> Triggers hardware fail-safes in all nodes if RC signal drops.