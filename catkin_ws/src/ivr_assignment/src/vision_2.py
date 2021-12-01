#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


class vision_2:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named vision 1
    rospy.init_node('vision_2', anonymous=True)

    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)

    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    
    self.end_eff_pub = rospy.Publisher("end_eff",Float64MultiArray, queue_size=10)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # store image 1 from camera 1
    self.image1 = []
    # store image 2 from camera 2
    self.image2 = []


  # Detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the red colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      # Isolate the yellow colour in the image as a binary image
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the green circle
  def detect_green(self,image):
      # Isolate the green colour in the image as a binary image
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  def callback1(self,data):
    # Recieve the image
    try:
      self.image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


  def callback2(self,data):
    # Recieve the image
    try:
      self.image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    a = self.detect_joint_angles()
    cv2.imshow('Image 1', self.image1)
    cv2.imshow('Image 2', self.image2)
    self.joints = Float64MultiArray()
    self.joints.data = a
    print(a)

    # detect robot end-effector from the image
    self.end_eff = Float64MultiArray()
    self.end_eff.data = self.detect_end_effector()


    cv2.waitKey(1)

    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.image1, "bgr8"))
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.image2, "bgr8"))
      self.joints_pub.publish(self.joints)
      self.end_eff_pub.publish(self.end_eff)

    except CvBridgeError as e:
      print(e)


  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_yellow(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 4 / np.sqrt(dist)
  
  def detect_end_effector(self):
    camera_1 = self.pixel2meter(self.image1)
    camera_2 = self.pixel2meter(self.image2)

    # Obtain the centre of each coloured blob 
    redYZ = camera_1 * self.detect_red(self.image1)
    redXZ = camera_2 * self.detect_red(self.image2)
    red_blob = np.array([redXZ[0], redYZ[0], - redXZ[1]])
  
    greenYZ = camera_1 * self.detect_green(self.image1)
    greenXZ = camera_2 * self.detect_green(self.image2)
    green_blob = np.array([greenXZ[0], greenYZ[0], - greenXZ[1]])
    
    endPos = red_blob - green_blob
    return endPos

  def detect_joint_angles(self):
      camera_1 = self.pixel2meter(self.image1)
      camera_2 = self.pixel2meter(self.image2)

      # Obtain the centre of each coloured blob 
      redYZ = camera_1 * self.detect_red(self.image1)
      redXZ = camera_2 * self.detect_red(self.image2)
      red_blob = np.array([redXZ[0], redYZ[0], - redXZ[1]])

      blueYZ = camera_1 * self.detect_blue(self.image1)
      blueXZ = camera_2 * self.detect_blue(self.image2)
      blue_blob = np.array([blueXZ[0], blueYZ[0], - blueXZ[1]])

      yellowYZ = camera_1 * self.detect_yellow(self.image1)
      yellowXZ = camera_2 * self.detect_yellow(self.image2)
      yellow_blob = np.array([yellowXZ[0], yellowYZ[0], - yellowXZ[1]])


      # calculate reference for initial blue blob position
      blue_reference_1 = blue_blob - yellow_blob
      # calculate reference for initial red blob position
      red_reference_1 = red_blob - yellow_blob


      if blue_reference_1[1] > 0:
        joint_angle_1 = np.arctan2(blue_reference_1[0], blue_reference_1[1])
      elif blue_reference_1[0] > 0:
        joint_angle_1 = np.pi / 2
      else:
        joint_angle_1 = - np.pi / 2

      # calculate joint 1 rotation matrix as it rotates in the z-axis
      joint_1_rotation_matrix = np.array([
        [np.cos(joint_angle_1), -np.sin(joint_angle_1), 0],
        [np.sin(joint_angle_1), np.cos(joint_angle_1), 0],
        [0,0,1]
      ])

      # calculate reference for new blue blob position
      blue_reference_2 = np.matmul(joint_1_rotation_matrix, blue_reference_1)
      # calculate reference for new red blob position
      red_reference_2 = np.matmul(joint_1_rotation_matrix, red_reference_1)


      if blue_reference_2[2] > 0:
        joint_angle_3 = np.arctan2(blue_reference_2[1], blue_reference_2[2])
      elif blue_reference_2[1] > 0:
        joint_angle_3 = np.pi / 2
      else:
        joint_angle_3 = - np.pi / 2

      # calculate joint 3 rotation matrix as it rotates in the x-axis
      joint_3_rotation_matrix = np.array([
        [1,0,0],
        [0, np.cos(joint_angle_3), -np.sin(joint_angle_3)],
        [0, np.sin(joint_angle_3), np.cos(joint_angle_3)]
      ])

      # calculate reference for new blue blob position
      blue_reference_3 = np.matmul(joint_3_rotation_matrix, blue_reference_2)
      # calculate reference for new red blob position
      red_reference_3 = np.matmul(joint_3_rotation_matrix, red_reference_2) - blue_reference_2


      if red_reference_3[2] > 0:
        joint_angle_4 = np.arctan2(red_reference_3[0], red_reference_3[2])
      elif red_reference_3[0] > 0:
        joint_angle_4 = np.pi / 2
      else:
        joint_angle_4 = - np.pi / 2

      # Solve using trigonometry
      # joint_angle_2 = np.arctan2(centerYZ[0]- blueYZ[0], centerYZ[1] - blueYZ[1])
      # joint_anle_3 = np.arctan2(centerXZ[0]- blueXZ[0], centerXZ[1] - blueXZ[1])
      # joint_angle_4 = np.arctan2(redYZ[0]-blueYZ[0], redYZ[1]-blueYZ[1]) - ja2
      
      return np.array([joint_angle_1, joint_angle_3, joint_angle_4])

# call the class
def main(args):
  ic = vision_2()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)