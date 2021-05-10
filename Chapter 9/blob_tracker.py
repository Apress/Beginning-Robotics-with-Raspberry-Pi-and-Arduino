import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# setup detector and parameters
params = cv2.SimpleBlobDetector_Params()

params.filterByColor = False
params.filterByArea = True
params.minArea = 20000
params.maxArea = 30000
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = True
params.minCircularity = 0.5
params.maxCircularity = 1

det = cv2.SimpleBlobDetector_create(params)

# define blue
lower_blue = np.array([80,60,20])
upper_blue = np.array([130,255,255])

while True:
    ret, frame = cap.read()
    
    imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    blueMask = cv2.inRange(imgHSV,lower_blue,upper_blue)
    blur= cv2.blur(blueMask, (10,10))

    res = cv2.bitwise_and(frame, frame, mask=blueMask)
    
    # get and draw keypoint
    keypnts = det.detect(blur)

    cv2.drawKeypoints(frame, keypnts, frame, (0,0,255),
                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', blur)

    for k in keypnts:
        print k.size
     
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
