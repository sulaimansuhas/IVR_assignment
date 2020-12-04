#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
import vision
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
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

    #initialize publiesher to send coordinates from camera 1

    self.camera1b_xy_pub = rospy.Publisher("camera1b_xy", Float64MultiArray, queue_size=10)
    self.camera1g_xy_pub = rospy.Publisher("camera1g_xy", Float64MultiArray, queue_size = 10)
    self.camera1r_xy_pub = rospy.Publisher("camera1r_xy", Float64MultiArray, queue_size = 10)
    self.camera1y_xy_pub = rospy.Publisher("camera1y_xy", Float64MultiArray, queue_size = 10)

    self.camera2b_xy = rospy.Subscriber("camera2b_xy", Float64MultiArray, self.callbackc2b)
    self.camera2g_xy = rospy.Subscriber("camera2g_xy", Float64MultiArray, self.callbackc2g)
    self.camera2r_xy = rospy.Subscriber("camera2r_xy", Float64MultiArray, self.callbackc2r)
    self.camera2y_xy = rospy.Subscriber("camera2y_xy", Float64MultiArray, self.callbackc2y)
    self.camera2o_xy = rospy.Subscriber("camera2o_xy", Float64MultiArray, self.callbackc2o)
    self.camera2bf_xy = rospy.Subscriber("camera2bf_xy", Float64MultiArray, self.callbackc2bf)

    self.dist_o2bf_pub = rospy.Publisher("dist_o2bf", Float64, queue_size = 10)
    self.ja1_pub = rospy.Publisher("ja1", Float64, queue_size = 10)
    self.ja2_pub = rospy.Publisher("ja2", Float64, queue_size = 10)
    self.ja3_pub = rospy.Publisher("ja3", Float64, queue_size = 10)
    self.ja4_pub = rospy.Publisher("ja4", Float64, queue_size = 10)

    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    #initial time
    self.initial_time = rospy.get_time()

    #variables for storing previous xy values of camera 1 incase of error, 
    self.c1b_prev = np.array([0,0], dtype=np.float64)
    self.c1g_prev = np.array([0,0], dtype=np.float64)
    self.c1r_prev = np.array([0,0], dtype=np.float64)
    self.c1y_prev = np.array([0,0], dtype=np.float64)
    self.c1o_prev = np.array([0,0], dtype=np.float64)
    #variables for storing previous xy values of camera 2 incase of error
    self.c2b_prev = np.array([0,0], dtype=np.float64)
    self.c2g_prev = np.array([0,0], dtype=np.float64)
    self.c2r_prev = np.array([0,0], dtype=np.float64)
    self.c2y_prev = np.array([0,0], dtype=np.float64)
    self.c2o_prev = np.array([0,0], dtype=np.float64)


    #variables to store the incoming xy values from image2.py
    self.c2b_xy = np.array([0,0], dtype=np.float64)
    self.c2g_xy = np.array([0,0], dtype=np.float64)
    self.c2r_xy = np.array([0,0], dtype=np.float64)
    self.c2y_xy = np.array([0,0], dtype=np.float64)
    self.c2o_xy = np.array([0,0], dtype=np.float64)
    self.c2bf_xy = np.array([0,0], dtype=np.float64)


  def calculate_joint_trajectory(self):
    current_time = np.array([rospy.get_time() - self.initial_time])
    j1 = 0
    j2 = float((np.pi/2) * np.sin((np.pi/15)*current_time))
    j3 = float((np.pi/2) * np.sin((np.pi/18)*current_time))
    j4 = float((np.pi/2) * np.sin((np.pi/20)*current_time))
    return np.array([j1, j2, j3, j4])

  # these functions get the camera 2 xy values from the subcscriber
  def callbackc2b(self, c2b):
    self.c2b_xy = c2b.data
    self.c2b_xy = np.array(self.c2b_xy).flatten()

  def callbackc2g(self,c2g):
    self.c2g_xy = c2g.data
    self.c2g_xy = np.array(self.c2g_xy).flatten()

  def callbackc2r(self,c2r):
    self.c2r_xy = c2r.data
    self.c2r_xy = np.array(self.c2r_xy).flatten()
  def callbackc2y(self,c2y):
    self.c2y_xy = c2y.data
    self.c2y_xy = np.array(self.c2y_xy).flatten()
  def callbackc2o(self,c2o):
    self.c2o_xy = c2o.data
    self.c2o_xy = np.array(self.c2o_xy).flatten()
  def callbackc2bf(self,c2bf):
    self.c2bf_xy = c2bf.data
    self.c2bf_xy = np.array(self.c2bf_xy).flatten()

  def threedee_coordinates(self,cam1coords,cam2coords, color,p2m):
    zeroes = np.array([0,0], dtype = np.float64)
    ones = np.array([1,1], dtype = np.float64)
    x=None
    y=None
    z1=None
    z2=None

    if color == 'g':
      print('!!!')
      print(cam1coords)                                                                                                                    #xy values of the green blob can't exceed for than the link length(3.5)
      if (np.array_equal(cam1coords, zeroes)) or (np.array_equal(cam1coords,ones)) or ((cam1coords[0]*p2m) > 3.5) or ((cam1coords[0]*p2m) < -3.5):  #if our estimate exceeds 3.5 or -3.5 due to a bit of error
        y = self.c1g_prev[0]
        z1 = self.c1g_prev[1]
      else:
        y = cam1coords[0]
        z1 = cam1coords[1]
        self.c1g_prev = cam1coords
      if (np.array_equal(cam2coords, zeroes)) or (np.array_equal(cam2coords,ones)) or ((cam2coords[0]*p2m) > 3.5) or ((cam2coords[0] *p2m) < -3.5): #same
        x = self.c2g_prev[0]
        z2 = self.c2g_prev[1]
      else:
        x = cam2coords[0]
        z2 = cam2coords[1]
        self.c2g_prev = cam2coords
    elif color == 'b':
      if (np.array_equal(cam1coords, zeroes)) or (np.array_equal(cam1coords,ones))  : 
        y = self.c1b_prev[0]
        z1 = self.c1b_prev[1]
      else:
        y = cam1coords[0]
        z1 = cam1coords[1]
        self.c1b_prev = cam1coords
      if (np.array_equal(cam2coords, zeroes)) or (np.array_equal(cam2coords,ones)):
        x = self.c2b_prev[0]
        z2 = self.c2b_prev[1]
      else:
        x = cam2coords[0]
        z2 = cam2coords[1]
        self.c2b_prev = cam2coords
    elif color == 'r':
      if (np.array_equal(cam1coords, zeroes)) or (np.array_equal(cam1coords,ones)) or ((cam1coords[0]*p2m) > 6.5) or ((cam1coords[0]*p2m) < -6.5):  #red can't exceed 3.5+3  
        y = self.c1r_prev[0]
        z1 = self.c1r_prev[1]
      else:
        y = cam1coords[0]
        z1 = cam1coords[1]
        self.c1r_prev = cam1coords
      if (np.array_equal(cam2coords, zeroes)) or (np.array_equal(cam2coords,ones)) or ((cam1coords[0]*p2m) > 6.5) or ((cam1coords[0]*p2m) < -6.5):
        x = self.c2r_prev[0]
        z2 = self.c2r_prev[1]
      else:
        x = cam2coords[0]
        z2 = cam2coords[1]
        self.c2r_prev = cam2coords
    elif color == 'y':
      if (np.array_equal(cam1coords, zeroes)) or (np.array_equal(cam1coords,ones)):   
        y = self.c1y_prev[0]
        z1 = self.c1y_prev[1]
      else:
        y = cam1coords[0]
        z1 = cam1coords[1]
        self.c1y_prev = cam1coords
      if (np.array_equal(cam2coords, zeroes)) or (np.array_equal(cam2coords,ones)):
        x = self.c2y_prev[0]
        z2 = self.c2y_prev[1]
      else:
        x = cam2coords[0]
        z2 = cam2coords[1]
        self.c2y_prev = cam2coords
    elif color == 'o':
      if (np.array_equal(cam1coords, zeroes)) or (np.array_equal(cam1coords,ones)):   
        y = self.c1o_prev[0]
        z1 = self.c1o_prev[1]
      else:
        y = cam1coords[0]
        z1 = cam1coords[1]
        self.c1o_prev = cam1coords
      if (np.array_equal(cam2coords, zeroes)) or (np.array_equal(cam2coords,ones)):
        x = self.c2o_prev[0]
        z2 = self.c2o_prev[1]
      else:
        x = cam2coords[0]
        z2 = cam2coords[1]
        self.c2o_prev = cam2coords
    elif color == 'bf': #we don't expect errors from the base frame so it's simpler to convert to 3d
      y = cam1coords[0]
      z1 = cam1coords[1]
      x = cam2coords[0]
      z2 = cam2coords[1]


    z=(z1+z2)/2 #incase of any difference between cam1 and cam2 (should be negligible)
    return np.array([x,y,z])

  def get_distance(self, point1,point2):  #gives distance from point1 to point2(for 3 points)
    return np.linalg.norm(point1-point2)

  # def pixel2meter(self,yellow, blue):
  #   return 2.5/(blue[1]-yellow[1]) #blue and yellows centroids should be more or less remain stationary. We know the length of the link is 2.5 m and the link is along the z axis
    #we have to pass the adjuested coordinates otherwise we get negative number

  def adjustcoordinates(self, yellow,blue,green,red): # since opencv coordinates start from the top left of the screen, we have to adjust to have the yellow joint as 0,0
    nb = blue-yellow
    nb[1] = nb[1] * - 1  # because we want our y coordinates to go upwards

    ng = green - yellow
    ng[1] = ng[1] * -1


    nr = red - yellow
    nr[1] = nr[1] * -1

    ny = yellow - yellow
    return ny, nb, ng, nr
  # Recieve data from camera 1, process it, and publish
  
  def calculate_joints_2_3_4(self, greenpos, redpos, bluepos): # we need the position of the green joint for joints angles 2 and 3, the position of the red joint is needed for  joint angle 4
    print("---------joint calculation debug starts here-----------")
    # if(greenpos[0]>3.5):
    #   dif = greenpos[0] -3.5
    #   greenpos[0] = 3.5 - dif
    # elif(greenpos[0]<-3.5):
    #   dif = -3.5 - greenpos[0]
    #   greenpos[0] = -3.5 + dif
    j3 = np.arcsin(greenpos[0]/3.5) #arcsin of x coordinate of the green joint divided by length of link 3
    print(greenpos[0])
    print(j3)
    y_z_dif = greenpos - bluepos
    j2 = np.arctan2(-y_z_dif[1], y_z_dif[2])
    print(np.cos(j3))
    print(greenpos[1])
    print(j2)
    

    redgreen_link = redpos-greenpos
    greenblue_link = greenpos - bluepos
    #projecting from redgreen link onto green blue link
    dotprod = redgreen_link[0] * greenblue_link[0] + redgreen_link[1] * greenblue_link[1] + redgreen_link[2] * greenblue_link[2]
    normvector = np.sqrt(greenblue_link[0]**2 + greenblue_link[1]**2 + greenblue_link[2]**2)
    projection = np.multiply((dotprod / normvector), greenblue_link)
    dotProductforcos = projection[0] * redgreen_link[0] + projection[1] * redgreen_link[1] + projection[2] * redgreen_link[2]
    normvec1 = np.sqrt(redgreen_link[0]**2+redgreen_link[1]**2+redgreen_link[2]**2)
    normvec2 = np.sqrt(projection[0]**2+projection[1]**2+projection[2]**2)
    ja4 = np.arccos(dotProductforcos / (normvec1 * normvec2)) #doesn't recognise a few quadrants here
    j4 = np.arctan2(np.sin(ja4), np.cos(ja4)) # so we use  arctan2
    if (redgreen_link[1] >= 0):
      j4 = -j4

    print(j4)

    print("---------ends here ----------------------------------")

    return j2,j3,j4

  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    print("baseframe")
    print(self.c2bf_xy)

    #calculating trajectories for sinusodal movement
    traj = self.calculate_joint_trajectory()
    self.j1_traj = Float64()
    self.j1_traj.data = traj[0]
    self.j2_traj = Float64()
    self.j2_traj.data = traj[1]
    self.j3_traj = Float64()
    self.j3_traj.data = traj[2]
    self.j4_traj = Float64()
    self.j4_traj.data = traj[3]

    #gettng the xy values for each blob from camera1
    g_xy , green_mask= vision.detect_green(self.cv_image1)
    # cv2.imshow('green_mask', green_mask)
    self.green = Float64MultiArray()
    self.green.data = g_xy
    b_xy , blue_mask= vision.detect_blue(self.cv_image1)
    # cv2.imshow('blue_mask', blue_mask)
    self.blue =Float64MultiArray()
    self.blue.data = b_xy
    r_xy, red_mask = vision.detect_red(self.cv_image1)
    # cv2.imshow('red_mask', red_mask)
    self.red = Float64MultiArray()
    self.red.data = r_xy
    y_xy, yellow_mask = vision.detect_yellow(self.cv_image1)
    # cv2.imshow('yellow_mask', yellow_mask)
    self.yellow = Float64MultiArray()
    self.yellow.data = y_xy

    adj_y2, adj_b2, adj_g2, adj_r2 = self.adjustcoordinates(self.c2y_xy,self.c2b_xy,self.c2g_xy, self.c2r_xy)

    adj_y1, adj_b1, adj_g1, adj_r1 = self.adjustcoordinates(y_xy, b_xy, g_xy, r_xy)

    print(adj_y1, adj_b1)
    p2m = 0.038461538461538464 # hardcoded pixel to meter ratio, obtained from pixel2meter.py file, hardcoded to reduce error
    o_xy , orange_mask= vision.detect_orange(self.cv_image1)
    # cv2.imshow('orange_mask', orange_mask)
    # cv2.drawContours(self.cv_image1, [contours], -1, (0,255,0), 1)
    pointo = cv2.circle(self.cv_image1, tuple(o_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)

    baseframe_xy = vision.detect_baseframe(self.cv_image1)
    pointbf = cv2.circle(self.cv_image1, tuple(baseframe_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)

    adj_o1 = o_xy - y_xy
    adj_o1[1] = adj_o1[1] * -1

    adj_o2 = self.c2o_xy - y_xy
    adj_o2[1] = adj_o2[1] * -1

    adj_baseframe1 = baseframe_xy - y_xy
    adj_baseframe1[1] = adj_baseframe1[1] * -1

    adj_baseframe2 = self.c2bf_xy - y_xy
    adj_baseframe2[1] = adj_baseframe2[1] * -1





    print(type(b_xy))
    print(y_xy)
    print("pixel2meter ratio:")
    print(p2m)
    self.green3d = (self.threedee_coordinates(adj_g1, adj_g2,'g',p2m))*p2m
    self.blue3d = (self.threedee_coordinates(adj_b1, adj_b2,'b',p2m))*p2m
    self.red3d = (self.threedee_coordinates(adj_r1, adj_r2,'r',p2m))*p2m
    self.yellow3d = (self.threedee_coordinates(adj_y1, adj_y2,'y',p2m))*p2m
    self.orange3d = (self.threedee_coordinates(adj_o1, adj_o2,'o',p2m))*p2m
    self.baseframe3d = (self.threedee_coordinates(adj_baseframe1, adj_baseframe2,'bf',p2m))*p2m

    print(self.baseframe3d)
    print(self.yellow3d)
    print(self.blue3d)

    self.o2bf = self.get_distance(self.orange3d,self.baseframe3d)

    print("o2bf distance")
    print(self.o2bf)


    self.ja2, self.ja3, self.ja4 = self.calculate_joints_2_3_4(self.green3d, self.red3d, self.blue3d)

    
    pointy = cv2.circle(self.cv_image1, tuple(y_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)
    pointb = cv2.circle(self.cv_image1, tuple(b_xy.astype(int)), radius=0, color=(0, 0, 255), thickness=3)
    pointg = cv2.circle(self.cv_image1, tuple(g_xy.astype(int)), radius=0, color=(0, 0, 0), thickness=3)
    pointr = cv2.circle(self.cv_image1, tuple(r_xy.astype(int)), radius=0, color=(255, 0, 0), thickness=3)


    # Uncomment if you want to save the image
    # cv2.imwrite('image_copy.png', self.cv_image1)
    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joint1_trajectory_pub.publish(self.j1_traj)
      self.joint2_trajectory_pub.publish(self.j2_traj)
      self.joint3_trajectory_pub.publish(self.j3_traj)
      self.joint4_trajectory_pub.publish(self.j4_traj)
      self.robot_joint1_pub.publish(self.j1_traj)
      self.robot_joint2_pub.publish(self.j2_traj)
      self.robot_joint3_pub.publish(self.j3_traj)
      self.robot_joint4_pub.publish(self.j4_traj)
      self.camera1r_xy_pub.publish(self.red)
      self.camera1g_xy_pub.publish(self.green)
      self.camera1b_xy_pub.publish(self.blue)
      self.camera1y_xy_pub.publish(self.yellow)
      self.dist_o2bf_pub.publish(self.o2bf)
      self.ja1_pub.publish(0)
      self.ja2_pub.publish(self.ja2)
      self.ja3_pub.publish(self.ja3)
      self.ja4_pub.publish(self.ja4)

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


