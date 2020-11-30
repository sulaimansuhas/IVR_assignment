#!/usr/bin/env python3

import roslib
import sys
import cv2
import numpy as np
import message_filter
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError

class image_converter:


  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('main', anonymous=True)
    # initialize publishers to send images coming from both of the cameras
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
    # initialize subscriber for both images time synchronized
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
    self.ts.registerCallback(self.callback2)
    # initialize publishers to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

	#initialize pulblishers to send joint trajectories

	self.joint1_trajectory_pub = rospy.Publisher("joint1_trajectory",Float64, queue_size=10)
    self.joint2_trajectory_pub = rospy.Publisher("joint2_trajectory",Float64, queue_size=10)
    self.joint3_trajectory_pub = rospy.Publisher("joint3_trajectory",Float64, queue_size=10)
    self.joint4_trajectory_pub = rospy.Publisher("joint4_trajectory",Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # intial time
    self.initial_time = rospy.get_time()



    def calculate_joint_trajectory(self):
    current_time = np.array([rospy.get_time() - self.initial_time])
    joint1 = 0
    joint2 = float((np.pi/2) * np.sin((np.pi/15)*current_time))
    joint3 = float((np.pi/2) * np.sin((np.pi/18)*current_time))
    joint4 = float((np.pi/2) * np.sin((np.pi/20)*current_time))
    return np.array([joint1, joint2, joint3, joint4])

    