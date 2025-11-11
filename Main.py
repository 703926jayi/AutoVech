#Setup Code here including initialization and definition
import Jetson.GPIO as GPIO
import time
from Throttle import Throttle
from Brakes import Brakes

def main():
    GPIO.setmode(GPIO.BOARD)
    servo = Throttle(pin=33)
    actuator = Brakes(dir_pin1=33, dir_pin2=35, enable_pin=31)

    try:
        target_position = 50.0
        Kp = 0.5

        while True:
            # Example actuator control loop
            adc_val = actuator.read_adc()
            position_pct = (adc_val / 1023.0) * 100.0
            print(f"Position: {position_pct:.1f}%")

            error = target_position - position_pct
            direction = 'extend' if error > 0 else 'retract'
            speed = min(abs(error) * Kp, 100)

            if abs(error) < 2.0:
                actuator.stop()
            else:
                actuator.move(direction, speed)

            # Example servo motion
            for angle in range(0, 181, 30):
                servo.set_angle(angle)
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        servo.cleanup()
        actuator.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    main()

