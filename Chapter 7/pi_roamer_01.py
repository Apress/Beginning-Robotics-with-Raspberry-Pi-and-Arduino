import serial
import time
import random

from adafruit_motorkit import MotorKit

# create motor object
kit = MotorKit()

# open serial port
ser = serial.Serial('/dev/ttyACM0', 115200)

# create variables
# sensors
distMid = 0.0
distLeft = 0.0
distRight = 0.0

# motor multipliers
m1Mult = 1.0
m2Mult = 1.0
m3Mult = 1.0
m4Mult = 1.0

# distance threshold
distThresh = 12.0
distCutOff = 30.0

# speeds
speedDef = 0.5
leftSpeed = speedDef
rightSpeed = speedDef
turnTime = 0.25
defTime = 0.1
driveTime = defTime

def driveMotors(leftChnl = speedDef, rightChnl = speedDef, duration = defTime):
    # determine the speed of each motor by multiplying
    # the channel by the motors multiplier
    m1Speed = leftChnl * m1Mult
    m2Speed = leftChnl * m2Mult
    m3Speed = rightChnl * m3Mult
    m4Speed = rightChnl * m4Mult

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

try:
    while 1:

# read the serial port
val = ser.readline().decode('utf=8')
print val

# parse the serial string
parsed = val.split(',')
parsed = [x.rstrip() for x in parsed]

# only assign new values if there are
# three or more available
if(len(parsed)>2):
    distMid = float(parsed[0] + str(0))
    distLeft = float(parsed[1] + str(0))
    distRight = float(parsed[2] + str(0))

# apply cutoff distance
if(distMid > distCutOff):
    distMid = distCutOff
if(distLeft > distCutOff):
    distLeft = distCutOff
if(distRight > distCutOff):
    distRight = distCutOff

# reset driveTime
driveTime = defTime

# if obstacle to left, steer right by increasing
# leftSpeed and running rightSpeed negative defSpeed
# if obstacle to right, steer to left by increasing
# rightSpeed and running leftSpeed negative
if(distLeft <= distThresh):
    leftSpeed = speedDef
    rightSpeed = -speedDef
elif (distRight <= distThresh):
    leftSpeed = -speedDef
    rightSpeed = speedDef
else:
    leftSpeed = speedDef
    rightSpeed = speedDef

# if obstacle dead ahead, stop then turn toward most
# open direction. if both directions open, turn random
if(distMid <= distThresh):
    # stop
    leftSpeed = 0
    rightSpeed = 0
    driveMotors(leftSpeed, rightSpeed, 1)
    time.sleep(1)
    leftSpeed = -150
    rightSpeed = -150
    driveMotors(leftSpeed, rightSpeed, 1)
    # determine preferred direction. if distLeft >
    # distRight, turn left. if distRight > distLeft,
    # turn right. if equal, turn random
    dirPref = distRight - distLeft
    if(dirPref == 0):
        dirPref = random.random()
    if(dirPref < 0):
        leftSpeed = -speedDef
        rightSpeed = speedDef
    elif(dirPref > 0):
        leftSpeed = speedDef
        rightSpeed = -speedDef
    driveTime = turnTime

# drive the motors
driveMotors(leftSpeed, rightSpeed, driveTime)

ser.flushInput()

except KeyboardInterrupt:
    kit.motor1.throttle(0)
    kit.motor2.throttle(0)
    kit.motor3.throttle(0)
    kit.motor4.throttle(0)
