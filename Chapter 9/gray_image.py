import cv2

img = cv2.imread('color_balls_small.jpg')
grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imshow('img', img)
cv2.imshow('gray', grayImg)

cv2.waitKey(0)
