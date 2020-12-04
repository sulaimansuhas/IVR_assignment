import cv2
import numpy as np

image = cv2.imread('pixel2meter.png')
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower=(100,100,0)
upper=(140,255,255)
mask_blue = cv2.inRange(image_hsv,lower,upper)
contours, hierarchy = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
M = cv2.moments(contours[0])
cbx = int(M['m10']/M['m00'])
cby = int(M['m01']/M['m00'])
# point = cv2.circle(image, (cbx,cby), radius=0, color=(0, 0, 255), thickness=3)
# cv2.imshow('mask_blue', image)
lower=(20,100,0)
upper=(32,255,255)
mask_yellow = cv2.inRange(image_hsv,lower,upper)
contours, hierarchy = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
M = cv2.moments(contours[0])
cyx = int(M['m10']/M['m00'])
cyy = int(M['m01']/M['m00'])
# point = cv2.circle(image, (cbx,cby), radius=0, color=(0, 0, 255), thickness=3)
# cv2.imshow('mask_blue', image)
# cv2.waitKey(0)
print(2.5/(cyy-cby))