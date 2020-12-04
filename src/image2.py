#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import vision
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # publishers for camera 2 coordinates
    self.camera2b_xy = rospy.Publisher("camera2b_xy", Float64MultiArray, queue_size=10)
    self.camera2g_xy = rospy.Publisher("camera2g_xy", Float64MultiArray, queue_size = 10)
    self.camera2r_xy = rospy.Publisher("camera2r_xy", Float64MultiArray, queue_size = 10)
    self.camera2y_xy = rospy.Publisher("camera2y_xy", Float64MultiArray, queue_size = 10)
    self.camera2o_xy = rospy.Publisher("camera2o_xy", Float64MultiArray, queue_size = 10)
    self.camera2bf_xy = rospy.Publisher("camera2bf_xy", Float64MultiArray, queue_size = 10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)



     #gettng the xy values for each blob from camera2
    g_xy , green_mask= vision.detect_green(self.cv_image2)
    # cv2.imshow('green_mask', green_mask)
    self.green = Float64MultiArray()
    self.green.data = g_xy
    b_xy , blue_mask= vision.detect_blue(self.cv_image2)
    # cv2.imshow('blue_mask', blue_mask)
    self.blue =Float64MultiArray()
    self.blue.data = b_xy
    r_xy, red_mask = vision.detect_red(self.cv_image2)
    # cv2.imshow('red_mask', red_mask)
    self.red = Float64MultiArray()
    self.red.data = r_xy
    y_xy, yellow_mask = vision.detect_yellow(self.cv_image2)
    # cv2.imshow('yellow_mask', yellow_mask)
    self.yellow = Float64MultiArray()
    self.yellow.data = y_xy

    #check for centroids
    # pointy = cv2.circle(self.cv_image2, tuple(y_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)
    # pointb = cv2.circle(self.cv_image2, tuple(b_xy.astype(int)), radius=0, color=(0, 0, 255), thickness=3)
    # pointg = cv2.circle(self.cv_image2, tuple(g_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)
    # pointr = cv2.circle(self.cv_image2, tuple(r_xy.astype(int)), radius=0, color=(255, 0, 0), thickness=3)

    o_xy , orange_mask= vision.detect_orange(self.cv_image2)
    self.orange = Float64MultiArray()
    self.orange.data = o_xy
    # cv2.imshow('orange_mask', orange_mask)
    # cv2.drawContours(self.cv_image1, [contours], -1, (0,255,0), 1)
    pointo = cv2.circle(self.cv_image2, tuple(o_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)

    baseframe_xy = vision.detect_baseframe(self.cv_image2)
    self.baseframe = Float64MultiArray()
    self.baseframe.data = baseframe_xy

    # pointbf = cv2.circle(self.cv_image2, tuple(baseframe_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)
    # Uncomment if you want to save the image
    cv2.imwrite('image_copy.png', self.cv_image2)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.camera2r_xy.publish(self.red)
      self.camera2g_xy.publish(self.green)
      self.camera2b_xy.publish(self.blue)
      self.camera2y_xy.publish(self.yellow)
      self.camera2o_xy.publish(self.orange)
      self.camera2bf_xy.publish(self.baseframe)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


