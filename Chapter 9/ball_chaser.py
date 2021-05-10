import cv2
import numpy as np
import time

from adafruit_motorkit import MotorKit

# create motor objects
kit = MotorKit()

# create motor list
motors = [kit.motor1, kit.motor2, kit.motor3, kit.motor4]

# motor multipliers
motorMultiplier = [1.0, 1.0, 1.0, 1.0, 1.0]

# motor speeds
motorSpeed = [0,0,0,0]

# speeds
speedDef = 1.0
leftSpeed = speedDef
rightSpeed = speedDef
diff= 0
maxDiff = 0.5
turnTime = 0.5

# create camera object
cap = cv2.VideoCapture(0)
time.sleep(1)

# PID
kp = 1.0
ki = 1.0
kd = 1.0
ballX = 0.0
ballY = 0.0

x = {'axis':'X',
     'lastTime':int(round(time.time()*1000)),
     'lastError':0.0,
     'error':0.0,
     'duration':0.0,
     'sumError':0.0,
     'dError':0.0,
     'PID':0.0}
y = {'axis':'Y',
     'lastTime':int(round(time.time()*1000)),
     'lastError':0.0,
     'error':0.0,
     'duration':0.0,
     'sumError':0.0,
     'dError':0.0,
     'PID':0.0}

# setup detector
params = cv2.SimpleBlobDetector_Params()

# define detector parameters
params.filterByColor = False
params.filterByArea = True
params.minArea = 15000
params.maxArea = 40000
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = True
params.minCircularity = 0.5
params.maxCircularity = 1

# create blob detector object
det = cv2.SimpleBlobDetector_create(params)

# define blue
lower_blue = np.array([80,60,20])
upper_blue = np.array([130,255,255])

def driveMotors(leftChnl = speedDef, rightChnl = speedDef,
                duration = defTime):
    # determine the speed of each motor by multiplying
    # the channel by the motors multiplier
    motorSpeed[0] = leftChnl * motorMultiplier[0]
    motorSpeed[1] = leftChnl * motorMultiplier[1]
    motorSpeed[2] = rightChnl * motorMultiplier[2]
    motorSpeed[3] = rightChnl * motorMultiplier[3]

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

def PID(axis):
    lastTime = axis['lastTime']
    lastError = axis['lastError']

    # get the current time
    now = int(round(time.time()*1000))
    duration = now-lastTime

    # calculate the error
    axis['sumError'] += axis['error'] * duration
    axis['dError'] = (axis['error'] - lastError)/duration

    # prevent runaway values
    if axis['sumError'] > 1:axis['sumError'] = 1
    if axis['sumError'] < -1: axis['sumError'] = -1

    # calculate PID
    axis['PID'] = kp * axis['error'] + ki * axis['sumError'] + kd * axis['dError']

    # update variables
    axis['lastError'] = axis['error']
    axis['lastTime'] = now

    # return the output value
    return axis

def killMotors():
    motors[0].throttle(0)
    motors[1].throttle(0)
    motors[2].throttle(0)
    motors[3].throttle(0)

# main program
try:
    while True:
        # capture video frame
        ret, frame = cap.read()

        # calculate center of frame
        height, width, chan = np.shape(frame)
        xMid = width/2 * 1.0
        yMid = height/2 * 1.0

        # filter image for blue ball
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blueMask = cv2.inRange(imgHSV, lower_blue, upper_blue)
        blur = cv2.blur(blueMask, (10,10))

        res = cv2.bitwise_and(frame,frame,mask=blur)

        # get keypoints
        keypoints = det.detect(blur)
        try:
            ballX = int(keypoints[0].pt[0])
            ballY = int(keypoints[0].pt[1])
        except:
            pass

        # draw keypoints
        cv2.drawKeypoints(frame, keypoints, frame, (0,0,255),
                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # calculate error and get PID ratio
        xVariance = (ballX - xMid) / xMid
        yVariance = (yMid - ballY) / yMid

        x['error'] = xVariance/xMid
        y['error'] = yVariance/yMid

        x = PID(x)
        y = PID(y)

        # calculate left and right speeds
        leftSpeed = (speedDef * y['PID']) + (maxDiff * x['PID'])
        rightSpeed = (speedDef * y['PID']) - (maxDiff * x['PID'])

        # another safety check for runaway values
        if leftSpeed > (speedDef + maxDiff): leftSpeed = (speedDef + maxDiff)
        if leftSpeed < -(speedDef + maxDiff): leftSpeed = -(speedDef + maxDiff)
        if rightSpeed > (speedDef + maxDiff): rightSpeed = (speedDef + maxDiff)
        if rightSpeed < -(speedDef + maxDiff): rightSpeed = -(speedDef + maxDiff)

        # drive motors
        driveMotors(leftSpeed, rightSpeed, driveTime)

        # show frame
##        cv2.imshow('frame', frame)
##        cv2.waitKey(1)

except KeyboardInterrupt:
    killMotors()
    cap.release()
    cv2.destroyAllWindows()
