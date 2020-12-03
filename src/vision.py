import cv2
import numpy as np

# detect the yellow blob
def detect_yellow(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower=(20, 100, 0)
	upper=(30, 255, 255)
	mask_yellow = cv2.inRange(image,lower,upper)
	contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	if len(contours)>0: #If the joint is visible
			largest_contour = max(contours, key=lambda cont: cv2.contourArea(cont))
			M = cv2.moments(largest_contour)
			# print(M)
			if int(M['m00'])!= 0 :
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])	
				return np.array([cx,cy], dtype = np.float64),mask_yellow,
			else: #just for some weird cases where M0 = 0to avoid division error 
				return np.array([1,1], dtype=np.float64),mask_yellow,  #chose 1,1 for debugging purposes
	else:
		return np.array([0,0], dtype=np.float64),mask_yellow,  # if it's not visible at all  # if it's not visible at all


def detect_blue(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower=(100, 150, 0)
	upper=(140,255,255)
	mask_blue = cv2.inRange(image,lower,upper)
	contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# print(largest_contour)
	if len(contours)>0: #If the joint is visible
			largest_contour = max(contours, key=lambda cont: cv2.contourArea(cont))
			M = cv2.moments(largest_contour)
			# print(M)
			if int(M['m00'])!= 0 :
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])	
				return np.array([cx,cy], dtype = np.float64),mask_blue
			else: #just for some weird cases where M0 = 0to avoid division error 
				return np.array([1,1], dtype=np.float64),mask_blue
	else:
		return np.array([0,0], dtype=np.float64),mask_blue # if it's not visible at all  # if it's not visible at all



def detect_red(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower = (0, 70, 50)
	upper = (10, 255, 255)
	mask_red = cv2.inRange(image,lower,upper)
	contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# print(largest_contour)
	if len(contours)>0: #If the joint is visible
			largest_contour = max(contours, key=lambda cont: cv2.contourArea(cont))
			M = cv2.moments(largest_contour)
			# print(M)
			if int(M['m00'])!= 0 :
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])	
				return np.array([cx,cy], dtype = np.float64),mask_red
			else: #just for some weird cases where M0 = 0to avoid division error 
				return np.array([1,1], dtype=np.float64),mask_red
	else:
		return np.array([0,0], dtype=np.float64),mask_red # if it's not visible at all  # if it's not visible at all



def detect_green(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower = (36, 25, 25)
	upper = (70, 255, 255)
	mask_green = cv2.inRange(image,lower,upper)
	contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# print(largest_contour)
	if len(contours)>0: #If the joint is visible
			largest_contour = max(contours, key=lambda cont: cv2.contourArea(cont))
			M = cv2.moments(largest_contour)
			# print(M)
			if int(M['m00'])!= 0 :
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])	
				return np.array([cx,cy], dtype = np.float64),mask_green
			else: #just for some weird cases where M0 = 0to avoid division error 
				return np.array([1,1], dtype=np.float64),mask_green
	else:
		return np.array([0,0], dtype=np.float64),mask_green # if it's not visible at all  # if it's not visible at all


def detect_orange(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower = (10,100,20)
	upper = (25, 255, 255)
	mask_orange = cv2.inRange(image,lower,upper)
	contours,_ = cv2.findContours(mask_orange,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	print(len(contours))
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
		print(len(approx))
		if len(approx)>=10:
			M = cv2.moments(approx)
			if int(M['m00'])!= 0 :
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])	
				return np.array([cx,cy], dtype = np.float64),mask_orange
			else: #just for some weird cases where M0 = 0to avoid division error 
				return np.array([1,1], dtype=np.float64),mask_orange
		else:
			#  we need this because if the circle is the smaller contour it won't get read as the for loop is exited once we put in a return statement
			M = cv2.moments(contours[1])
			if int(M['m00'])!= 0 :
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])	
				return np.array([cx,cy], dtype = np.float64),mask_orange
			else: #just for some weird cases where M0 = 0to avoid division error 
				return np.array([1,1], dtype=np.float64),mask_orange



def detect_baseframe(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower = (0, 0, 180)
	upper = (0, 0, 255)
	mask_baseframe = cv2.inRange(image,lower,upper)
	contours,_ = cv2.findContours(mask_baseframe,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	largest_contour = max(contours, key=lambda cont: cv2.contourArea(cont))
	M = cv2.moments(largest_contour)
	return np.array([int(M['m10']/M['m00']), int(M['m01']/M['m00'])], dtype = np.float64)

	






