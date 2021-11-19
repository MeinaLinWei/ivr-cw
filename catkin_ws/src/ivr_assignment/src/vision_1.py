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


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)

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
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    self.image1 = []
    self.image2 = []


  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
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
 

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def callback1(self,data):
    # Recieve the image
    try:
      self.image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.waitKey(3)


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

    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.image1, "bgr8"))
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.image2, "bgr8"))
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)

    
    cv2.waitKey(3)

  def detect_joint_angles(self):
      camera_1 = self.pixel2meter(self.image1)
      camera_2 = self.pixel2meter(self.image2)

      # Obtain the centre of each coloured blob 
      centerYZ = camera_1 * self.detect_yellow(self.image1)
      centerXZ = camera_2 * self.detect_yellow(self.image2)

      yellow_c = centerXZ[0], centerYZ[0], - centerXZ[1]

      blueYZ = camera_1 * self.detect_blue(self.image1)
      blueXZ = camera_2 * self.detect_blue(self.image2)

      blue_c = blueXZ[0], blueYZ[0], - blueXZ[1]

      redYZ = camera_1 * self.detect_red(self.image1)
      redXZ = camera_2 * self.detect_red(self.image2)

      red_c = redXZ[0], redYZ[0], - redXZ[1]

      blue_frame = blue_c - yellow_c
      red_frame = red_c - yellow_c

      if blue_frame[2] > 0:
        ja2 = np.arctan2(blue_frame[0], blue_frame[2])
        # ja2 = np.arctan2(centerYZ[0]- blueYZ[0], centerYZ[1] - blueYZ[1])
      elif blue_frame[0] > 0:
        ja2 = np.pi / 2
      else:
        ja2 = - np.pi / 2

      j2_rotation_mat = np.array([
        [np.cos(ja2), 0, np.sin(ja2)],
        [0,1,0],
        [np.sin(ja2), 0, np.cos(ja2)]
      ])

      blue_frame_ref2 = np.matmul(j2_rotation_mat, blue_frame)
      red_frame_ref2 = np.matmul(j2_rotation_mat, red_frame)

      opp = blue_frame_ref2[1]

      if blue_frame_ref2[2] > 0:
        ja3 = np.arctan2(blue_frame_ref2[1], blue_frame_ref2[2])
        # ja2 = np.arctan2(centerYZ[0]- blueYZ[0], centerYZ[1] - blueYZ[1])
      elif blue_frame_ref2[1] > 0:
        ja3 = np.pi / 2
      else:
        ja3 = - np.pi / 2

    
      j3_rotation_mat = np.array([
        [1,0,0],
        [0, np.cos(ja3), -np.sin(ja3)],
        [0, np.sin(ja3), np.cos(ja3)]
      ])

      blue_frame_ref3 = np.matmul(j3_rotation_mat, blue_frame_ref2)
      red_frame_ref3 = np.matmul(j3_rotation_mat, red_frame_ref2)

      opp = blue_frame_ref2[1]

      if blue_frame_ref3[2] > 0:
        ja4 = np.arctan2(blue_frame_ref3[0], blue_frame_ref3[2])
        # ja2 = np.arctan2(centerYZ[0]- blueYZ[0], centerYZ[1] - blueYZ[1])
      elif blue_frame_ref3[0] > 0:
        ja4 = np.pi / 2
      else:
        ja4 = - np.pi / 2

      # Solve using trigonometry
      # ja3 = np.arctan2(centerXZ[0]- blueXZ[0], centerXZ[1] - blueXZ[1])
      # ja4 = np.arctan2(redYZ[0]-blueYZ[0], redYZ[1]-blueYZ[1]) - ja2
      
      return np.array([ja2, ja3, ja4])
    
  
    # Recieve data, process it, and publish

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