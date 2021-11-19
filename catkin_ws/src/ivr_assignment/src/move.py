class move:
# Defines subscribers
    def __init__(self):
        # initialise the node named vision
        rospy.init_node('move', anonymous=True)
        # initialise a publisher to send the next joint 2 position to joint2_position_controller
        selt.joint_pub2 = rospy. Publisher( "/robot/joint2_position_controller/command", Float64, queue_size = 1)
        # initialise a publisher to send the next joint 3 position to joint2_position_controller
        self.joint_pub3 = rospy. Publisher( "/robot/joint3_position_controller/command", Float 64, queue_size = 1)
        # initialise a publisher to send the next joint 4 position to joint2_position_controller
        self.joint_pub4 = rospy.Publisher( "/robot/joint4_position_controller/command", Float 64, queue_size = 1)
        # Moves joints in sinusoidal fashion
    
    def move joints(self):
        # Fix the refresh rate to 10 HZ
        rate = rospy.Rate(10)
        # Run indefinetly
        while not rospy.is_shutdown():
            # Get current time elapsed
            t = rospy.get_time()
            # Calculate a next position for desired joints 2,3 and 4
            nextpoint2_pos = np.p1/2 * (np.sin(np.p1/15 t))
            next_joint3_pos = np.pi/2 * (np.sin(np.pi/20 t))
            next_joint4_pos = np.pi/2 * (np.sin(np.pi/18 t))
        # Publish the results
        try:
        # Publish a new position for joint 2,3 and 4
            self.joint_pub2.publish(next_joint2_pos)
            self.joint_pub3.publish(next_joint3_pos)
            self.joint_pub4.publish(next_joint4_pos)
        except CvBridgeError as e:
        
        print(e)
        rate. sleep()
        # call the class

    def main(args):
        m = Move()
        try:
            m.movejoints()
        except Keyboard Interrupt :
            print("Shutting down")
        CV2.destroyAllWindows()
        # run the code if the node is called

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
