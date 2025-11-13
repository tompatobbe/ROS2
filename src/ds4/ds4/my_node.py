#!/usr/bin/env python3
import rospy

def main():
    # Initialize the ROS node
    rospy.init_node('hello_node', anonymous=True)
    
    # Set a loop rate (1 Hz)
    rate = rospy.Rate(1)
    
    # Main loop
    while not rospy.is_shutdown():
        rospy.loginfo("Hello, ROS World!")
        rate.sleep()

# Standard Python boilerplate to run main() when executed
if __name__ == '__main__':
    main()
