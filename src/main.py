#!/usr/bin/env python3

import roslib
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:


	def __init__(self):
		# initialize the node named image_processing
		rospy.init_node('main', anonymous=True)
		# initialize publishers to send images coming from both of the cameras
		self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
		self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
		# initialize subscriber for both images time synchronized
		self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image, self.callback)
		self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image, self.callback)
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
		j1 = 0
		j2 = float((np.pi/2) * np.sin((np.pi/15)*current_time))
		j3 = float((np.pi/2) * np.sin((np.pi/18)*current_time))
		j4 = float((np.pi/2) * np.sin((np.pi/20)*current_time))
		return np.array([j1, j2, j3, j4])

	def callback(self,data1):
		traj = self.calculate_joint_trajectory()
		self.j1_traj = Float64()
		self.j1_traj.data = traj[0]
		self.j2_traj = Float64()
		self.j2_traj.data = traj[1]
		self.j3_traj = Float64()
		self.j3_traj.data = traj[2]
		self.j4_traj = Float64()
		self.j4_traj.data = traj[3]


		try:
			self.joint1_trajectory_pub.publish(self.j1_traj)
			self.joint2_trajectory_pub.publish(self.j2_traj)
			self.joint3_trajectory_pub.publish(self.j3_traj)
			self.joint4_trajectory_pub.publish(self.j4_traj)
			self.robot_joint1_pub.publish(self.j1_traj)
			self.robot_joint2_pub.publish(self.j2_traj)
			self.robot_joint3_pub.publish(self.j3_traj)
			self.robot_joint4_pub.publish(self.j4_traj)
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
