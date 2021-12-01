#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
    # initialize a publisher to send desired trajectory
    self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)
    
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    self.end_eff_sub = rospy.Subscriber("/end_eff",Float64MultiArray, self.callback1)
    self.joints_sub = rospy.Subscriber("/joints_pos",Float64MultiArray, self.callback2)
    self.target_sub = rospy.Subscriber("/target_control/target_pos",Float64MultiArray, self.callback3)


    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
    # initialize error and derivative of error for trajectory tracking
    self.error = np.array([0.0,0.0], dtype='float64')
    self.error_d = np.array([0.0,0.0], dtype='float64')

    self.end_eff = []
    self.joints = []



  # Calculate the forward kinematics
  # 0= 1 1=3 2=4
  def forward_kinematics(self, joints):
    l1, l2, l3 = 4.0, 3.2, 2.8

    end_effector = np.array(
        [l3 * np.sin(joints[0])*np.sin(joints[1])*np.cos(joints[2]) + l3 * np.cos(joints[0]) * np.sin(joints[2]) + l2 * np.sin(joints[0]) * np.sin(joints[1]),
        - l3 * np.cos(joints[0])*np.sin(joints[1])*np.cos(joints[2]) + l3 * np.sin(joints[0]) * np.sin(joints[2]) - l2 * np.cos(joints[0]) * np.sin(joints[1]),
        l3 * np.cos(joints[1])*np.cos(joints[2]) + l2 * np.cos(joints[1]) + l1  
        ])
    return end_effector

  
  def callback1(self,data):
    # Recieve the image
    try:
        self.end_eff = data.data
    except CvBridgeError as e:
        print(e)

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
        self.joints = data.data
    except CvBridgeError as e:
        print(e)
    
    end_calc = self.forward_kinematics(self.joints)

    print("END_CALC:")
    print(end_calc)
    print("END_IMG:")
    print(self.end_eff)

    
    # Define a circular trajectory
  def trajectory(self):
    # get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    x_d = float(5.5* np.cos(cur_time * np.pi/100))
    y_d = float(5.5 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    return np.array([x_d, y_d])

# Calculate the robot Jacobian
  def calculate_jacobian(self):
    l1, l2, l3 = 4.0, 3.2, 2.8
    joints = self.joints
    x11 = l3 * np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2]) - l3 * np.sin(joints[0]) * np.sin(joints[2]) + l2 * np.cos(joints[0]) * np.sin(joints[1])
    x12 = l3 * np.sin(joints[0]) * np.cos(joints[1]) * np.cos(joints[2]) + l2 * np.sin(joints[0]) * np.cos(joints[1])
    x13 = -l3 * np.sin(joints[0]) * np.sin(joints[1]) * np.sin(joints[2]) + l3 * np.cos(joints[0]) * np.cos(joints[2])


    x21 = l3 * np.sin(joints[0]) * np.sin(joints[1]) * np.cos(joints[2]) + l3 * np.cos(joints[0]) * np.sin(joints[2]) + l2 * np.sin(joints[0]) * np.sin(joints[1])
    x22 = -l3 * np.cos(joints[0]) * np.cos(joints[1]) * np.cos(joints[2]) - l2 * np.cos(joints[0]) * np.cos(joints[1])
    x23 = l3 * np.cos(joints[0]) * np.sin(joints[1]) * np.sin(joints[2]) + l3 * np.sin(joints[0]) * np.cos(joints[2])

    x31 = 0
    x32 = -l3 * np.sin(joints[1]) * np.cos(joints[2]) - l2 * np.sin(joints[1])
    x33 = -l3 * np.cos(joints[1]) * np.sin(joints[2]) 

    jacobian = np.array([[x11, x12, x13], [x21, x22, x23 ], [x31, x32, x33]])
    return jacobian


  # Estimate control inputs for open-loop control
  def control_open(self, target):
    # estimate time step
    cur_time = rospy.get_time()
    dt = cur_time - self.time_previous_step2
    self.time_previous_step2 = cur_time
    
    J_inv = np.linalg.pinv(self.calculate_jacobian())  # calculating the psudeo inverse of Jacobian
    
    # desired trajectory
    pos_d= target
    # estimate derivative of desired trajectory
    self.error = (pos_d - self.end_eff)/dt
    q_d = self.joints + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
    return q_d



  def callback3(self,data):
    # Recieve the image
    try:
        self.target = data
    except CvBridgeError as e:
        print(e)
        
    # send control commands to joints 
    q_d = self.control_open(self.target)
    '''
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint3=Float64()
    self.joint3.data= q_d[1]
    self.joint4=Float64()
    self.joint4.data= q_d[2]
    '''

    
    print("TARGET")
    print(self.target)

    print("CALC")
    print(q_d)

    # Publish the results
    '''
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
      self.end_effector_pub.publish(self.end_effector)
      self.trajectory_pub.publish(self.trajectory_desired)
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
    except CvBridgeError as e:
      print(e)
    '''


# call the class
def main(args):
  ic = control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
