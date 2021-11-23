#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class Move:
# Defines subscribers
    def __init__(self):
        # initialise the node named vision
        rospy.init_node('vision_2_move', anonymous=True)
        # initialise a publisher to send the next joint 2 position to joint2_position_controller
        self.joint_pub1 = rospy.Publisher( "/robot/joint1_position_controller/command", Float64, queue_size = 10)
        # initialise a publisher to send the next joint 3 position to joint2_position_controller
        self.joint_pub3 = rospy.Publisher( "/robot/joint3_position_controller/command", Float64, queue_size = 10)
        # initialise a publisher to send the next joint 4 position to joint2_position_controller
        self.joint_pub4 = rospy.Publisher( "/robot/joint4_position_controller/command", Float64, queue_size = 10)
    
        self.nextPositionJoint1 = Float64()
        self.nextPositionJoint3 = Float64()
        self.nextPositionJoint4 = Float64()

    def move_ja(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            time = rospy.get_time()
            self.nextPositionJoint1.data = np.pi/2 * (np.sin(np.pi/15 * time))
            self.nextPositionJoint3.data = np.pi/2 * (np.sin(np.pi/20 * time))
            self.nextPositionJoint4.data = np.pi/2 * (np.sin(np.pi/18 * time))
            try:
                self.joint_pub1.publish(self.nextPositionJoint1.data)
                self.joint_pub3.publish(self.nextPositionJoint3.data)
                self.joint_pub4.publish(self.nextPositionJoint4.data)
            except CvBridgeError as e:
                print(e)
        
            rate.sleep()
        # call the class

def main(args):
    m = Move()
    try:
        m.move_ja()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    # run the code if the node is called

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
