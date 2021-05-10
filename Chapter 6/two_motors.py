import time
from adafruit_motorkit import MotorKit

# create motor object
kit = MotorKit()

try:
    while True:
        # ramp up speed from 1 to 255
        for i in range(100):
            j = 100 - i

            # run motors forward 1 second
            kit.motor1.throttle = i / 100
            kit.motor2.throttle = j / 100

        # stop motor
        kit.motor1.throttle = 0.0
        kit.motor2.throttle = 0.0
        time.sleep(0.25)

        for i in reversed(range(100)):
            j = 100 - i

            # run motors reverse 1 second
            kit.motor1.throttle = i / 100
            kit.motor2.throttle = j / 100

        # stop motor
        kit.motor1.throttle = 0.0
        kit.motor2.throttle = 0.0
        time.sleep(0.25)

except KeyboardInterrupt:
    kit.motor1.throttle = 0.0
    kit.motor2.throttle = 0.0
