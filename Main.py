'''
DEPRECATED: DO NOT USE IN ROS 2 ARCHITECTURE
This script is a standalone legacy test file using Jetson.GPIO. 
It has been superseded by the ROS 2 Humble node architecture outlined in 
AutoVech Documentation (Software).md. 

Running this script will cause hardware resource conflicts
with the active autovech ROS 2 packages.

import Jetson.GPIO as GPIO
import time
from Throttle import Throttle
from Brakes import Brakes
from Gamepad import Gamepad

def main():
    GPIO.setmode(GPIO.BOARD)

    # Initialize hardware
    servo = Throttle(pin=33)
    actuator = Brakes(dir_pin1=33, dir_pin2=35, enable_pin=31)
    gamepad = Gamepad()


    print("Gamepad control active. Use triggers for actuator, left stick for servo.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            # Update controller inputs
            gamepad.update()

            # === Servo control ===
            # Map left stick Y (-1 to 1) → servo angle (0 to 180)
            angle = (1 - gamepad.left_stick_y) * 90  # center at 90°
            servo.set_angle(angle)

            # === Actuator control ===
            # Use triggers: left = retract, right = extend
            if gamepad.right_trigger > 0.1:
                actuator.move('extend', speed_pct=gamepad.right_trigger * 100)
            elif gamepad.left_trigger > 0.1:
                actuator.move('retract', speed_pct=gamepad.left_trigger * 100)
            else:
                actuator.stop()


            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting gracefully...")

    finally:
        servo.cleanup()
        actuator.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    main()'''

