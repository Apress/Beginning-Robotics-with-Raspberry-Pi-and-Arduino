
import time
from adafruit_motorkit import MotorKit

# create motor object
kit = MotorKit()

# run motor forward 1 second
kit.motor1.throttle = 1.0
time.sleep(1.0)

# stop motor
kit.motor1.throttle = 0.0
time.sleep(0.25)

# run motor reverse 1 second
kit.motor1.throttle = -1.0
time.sleep(1.0)

# stop motor
kit.motor1.throttle = 0.0



