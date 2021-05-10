import serial
import time

from adafruit_motorkit import MotorKit

# create motor object
kit = MotorKit()

# create motor list
motors = [kit.motor1, kit.motor2, kit.motor3, kit.motor4]

# motor multipliers
motorMultiplier = [1.0, 1.0, 1.0, 1.0, 1.0]

# motor speeds
motorSpeed = [0,0,0,0]

# open serial port
ser = serial.Serial('/dev/ttyAMA0', 9600)

# create variables
# sensors
irSensors = [0,0,0,0,0]
irMins = [0,0,0,0,0]
irMaxs = [0,0,0,0,0]
irThesh = 50

# speeds
speedDef = 1.0
leftSpeed = speedDef
rightSpeed = speedDef
corMinor = 0.25
corMajor = 0.5
turnTime = 0.5
defTime = 0.01
driveTime = defTime
sweepTime = 1000 #duration of a sweep in milliseconds

def driveMotors(leftChnl = speedDef, rightChnl = speedDef,
                duration = defTime):
    # determine the speed of each motor by multiplying
    # the channel by the motors multiplier
    motorSpeed[0] = leftChnl * motorMultiplier[0]
    motorSpeed[1] = leftChnl * motorMultiplier[1]
    motorSpeed[2] = rightChnl * motorMultiplier[2]
    motorSpeed[3] = rightChnl * motorMultiplier[3]

# set each motor speed. Since the speed can be a
# negative number, we take the absolute value
for x in range(4):
    motors[x].setSpeed(abs(int(motorSpeed[x])))

# run the motors. if the channel is negative, run
# reverse. else run forward
if(leftChnl < 0):
    motors[0].throttle(-motorSpeed[0])
    motors[1].throttle(-motorSpeed[1])
else:
    motors[0].throttle(motorSpeed[0])
    motors[1].throttle(motorSpeed[1])

if (rightChnl < 0):
    motors[2].throttle(motorSpeed[2])
    motors[3].throttle(motorSpeed[3])
else:
    motors[2].throttle(-motorSpeed[2])
    motors[3].throttle(-motorSpeed[3])

# wait for duration
time.sleep(duration)

def getIR():
    # read the serial port
    val = ser.readline().decode('utf-8')

    # parse the serial string
    parsed = val.split(',')
    parsed = [x.rstrip() for x in parsed]

if(len(parsed)==5):
    for x in range(5):
        irSensors[x] = int(parsed[x]+str(0))/10

# flush the serial buffer of any extra bytes
ser.flushInput()

def calibrate():
    # set up cycle count loop
    direction = 1
    cycle = 0

    # get initial values for each sensor
    # and set initial min/max values
    getIR()

    for x in range(5):
        irMins[x] = irSensors[x]
        irMaxs[x] = irSensors[x]

while cycle < 5:

    #set up sweep loop
    millisOld = int(round(time.time()*1000))
    millisNew = millisOld

while((millisNew-millisOld)<sweepTime):
    leftSpeed = speedDef * direction
    rightSpeed = speedDef * -direction

    # drive the motors
    driveMotors(leftSpeed, rightSpeed, driveTime)

    # read sensors
    getIR()

# set min and max values for each sensor
for x in range(5):
    if(irSensors[x] < irMins[x]):
        irMins[x] = irSensors[x]
    elif(irSensors[x] > irMaxs[x]):
        irMaxs[x] = irSensors[x]

millisNew = int(round(time.time()*1000))

# reverse direction
direction = -direction

# increment cycles
cycle += 1

# drive forward
driveMotors(speedDef, speedDef, driveTime)

def followLine():
    leftSpeed = speedDef
    rightSpeed = speedDef

    getIR()

    # find line and correct if necessary
    if(irMaxs[0]-irThresh <= irSensors[0]
<= irMaxs[0]+irThresh):
        leftSpeed = speedDef-corMajor
    elif(irMaxs[1]-irThresh <= irSensors[1]
<= irMaxs[1]+irThresh):
        leftSpeed = speedDef-corMinor
    elif(irMaxs[3]-irThresh <= irSensors[3]
<= irMaxs[3]+irThresh):
        rightSpeed = speedDef-corMinor
    elif(irMaxs[4]-irThresh <= irSensors[4]
<= irMaxs[4]+irThresh):
        rightSpeed = speedDef-corMajor
    else:
        leftSpeed = speedDef
        rightSpeed = speedDef

    # drive the motors
    driveMotors(leftSpeed, rightSpeed, driveTime)

# execute program
try:
    calibrate()

    while 1:
        followLine()
        time.sleep(0.01)

except KeyboardInterrupt:
    kit.motor1.throttle(0)
    kit.motor2.throttle(0)
    kit.motor3.throttle(0)
    kit.motor4.throttle(0)
