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

    self.image1 = None
    self.image2 = None
    self.joints = Float64MultiArray()
    
    a =  self.detect_joint_angles(self.image1, self.image2)
    self.joints.data = a

    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.image1, "bgr8"))
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.image2, "bgr8"))
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)




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


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_yellow(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 4 / np.sqrt(dist)


    # Calculate the relevant joint angles from the image
  def detect_joint_angles(self,imageX, imageY):
    camera_1 = self.pixel2meter(imageY)
    camera_2 = self.pixel2meter(imageX)

    # Obtain the centre of each coloured blob 
    centerYZ = camera_1 * self.detect_yellow(imageY)
    centerXZ = camera_2 * self.detect_yellow(imageX)

    blueYZ = camera_1 * self.detect_blue(imageY)
    blueXZ = camera_2 * self.detect_blue(imageX)

    redYZ = camera_1 * self.detect_red(imageY)
    redXZ = camera_2 * self.detect_red(imageX)


    # Solve using trigonometry
    ja2 = np.arctan2(centerYZ[0]- blueYZ[0], centerYZ[1] - blueYZ[1])
    ja3 = np.arctan2(centerXZ[0]- blueXZ[0], centerXZ[1] - blueXZ[1])
    ja4 = np.arctan2(redYZ[0]-blueYZ[0], redYZ[1]-blueYZ[1]) - ja2
    
    
    return np.array([ja2, ja3, ja4])
  
  # Recieve data, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.image1 = cv_image
    # Perform image processing task (your code goes here)
    # The image is loaded as cv_imag

    # Uncomment if you want to save the image
    # cv2.imwrite('cam1.png', cv_image)
    # image1 = cv2.imread("camera1.png",0)

    # a = self.detect_joint_angles("camera1.png", "camera2.png")
    cv2.imshow('window', self.image1)
    cv2.waitKey(3)

    # self.joints = Float64MultiArray()
    # self.joints.data = a

    # Publish the results
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #   self.joints_pub.publish(self.joints)
    # except CvBridgeError as e:
    #   print(e)

  def callback2(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.image2 = cv_image

    cv2.imshow('window', self.image1)
    cv2.waitKey(3)

  # def get_joint_angles(self):
  #   a =  self.detect_joint_angles(self.image1, self.image2)
    # self.joints = Float64MultiArray()
    # self.joints.data = a

    # Publish the results
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #   self.joints_pub.publish(self.joints)
    # except CvBridgeError as e:
    #   print(e)

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


