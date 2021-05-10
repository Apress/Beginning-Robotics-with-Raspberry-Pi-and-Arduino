import cv2
import numpy as np

cap = cv2.VideoCapture('test_video.avi')

while(True):
    ret,frame = cap.read()

    if ret:
        cv2.imshow('video', frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
