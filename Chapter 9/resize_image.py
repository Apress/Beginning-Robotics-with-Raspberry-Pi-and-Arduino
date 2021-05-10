import cv2

img = cv2.imread('color_balls_small.jpg')
x,y = img.shape[:2]
resImg = cv2.resize(img, (y/2, x/2), interpolation = cv2.INTER_AREA)

cv2.imshow('image', img)
cv2.imshow('resized', resImg)

cv2.waitKey(0)
