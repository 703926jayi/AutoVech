import can
import time
import struct

# -----------------------------
# User settings
ODRIVE_CAN_ID = 0      # Axis 0 on ODrive
BUS_NAME = "can0"
# -----------------------------

# CAN message IDs (ODrive S1)
MSG_SET_INPUT_VEL = 0x00 | (ODRIVE_CAN_ID << 5)

bus = can.interface.Bus(channel=BUS_NAME, bustype='socketcan')

def set_velocity(vel_rads):
    # float32 → 4 bytes, little-endian
    data = struct.pack("<f", vel_rads)
    msg = can.Message(arbitration_id=MSG_SET_INPUT_VEL, data=data, is_extended_id=False)
    bus.send(msg)


print("Motor forward...")
set_velocity(5.0)     # rad/s
time.sleep(3)

print("Stop...")
set_velocity(0.0)
time.sleep(1)

print("Motor backward...")
set_velocity(-5.0)
time.sleep(3)

print("Stop...")
set_velocity(0.0)

print("Done.")
