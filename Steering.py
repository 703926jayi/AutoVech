#Controlled through canbus conection
import can
import struct
import time

# CAN bus setup
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Spark MAX CAN ID (change if your Spark MAX uses a different ID)
SPARK_MAX_ID = 1

# Helper function to create a Spark MAX speed command
def spark_max_set_speed(speed: float):
    # Spark MAX expects float32 little-endian in data bytes
    speed_bytes = struct.pack('<f', speed)
    # 0x200 + device ID = standard command arbitration ID
    msg = can.Message(
        arbitration_id=0x200 + SPARK_MAX_ID,
        data=speed_bytes,
        is_extended_id=False
    )
    return msg

# Example usage: run motor at 50% for 5 seconds
try:
    print("Running motor at 50%")
    msg = spark_max_set_speed(0.5)
    bus.send(msg)

    time.sleep(5)

    print("Stopping motor")
    msg = spark_max_set_speed(0.0)
    bus.send(msg)

except can.CanError as e:
    print("CAN Error:", e)