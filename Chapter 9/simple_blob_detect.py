import cv2
import numpy as np

img = cv2.imread("color_balls_small.jpg")
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# setup parameters
params = cv2.SimpleBlobDetector_Params()

params.filterByColor = False
params.filterByArea = False
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = False

# create blob detector
det = cv2.SimpleBlobDetector_create(params)

lower_blue = np.array([80,120,120])
upper_blue = np.array([130,255,255])
 
blueMask = cv2.inRange(imgHSV,lower_blue,upper_blue)

res = cv2.bitwise_and(img, img, mask=blueMask)
res2 = res

# get keypoints
keypnts = det.detect(blueMask)

# draw keypoints
cv2.drawKeypoints(res, keypnts, res2, (0,0,255),
                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv2.imshow('img', img)
cv2.imshow('mask', blueMask)
cv2.imshow('blue', res)
cv2.imshow('res2', res2)

# print the coordinates and size of keypoints to terminal
for k in keypnts:
    print k.pt[0] 
    print k.pt[1]
    print k.size
 
cv2.waitKey(0)
