#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
import time

class DS4Listener:
    """
    ROS node to subscribe to the /joy topic (sensor_msgs/Joy) 
    and print the current state of axes and buttons.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ds4_listener', anonymous=True)

        # Set the subscription rate (to avoid flooding the terminal)
        self.publish_rate = rospy.Rate(5) # 5 Hz (prints 5 times per second)
        self.last_print_time = time.time()
        self.print_interval = 1.0 / self.publish_rate.desired_freq # 0.2 seconds

        # Subscriber to the /joy topic
        # The joy_node typically publishes on this topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.loginfo("DS4 Listener Node Initialized. Waiting for /joy messages...")

        # Store the last received message
        self.last_joy_msg = None

    def joy_callback(self, data):
        """
        Callback function for the /joy topic. Stores the latest message.
        """
        self.last_joy_msg = data
        
    def run(self):
        """
        The main loop that controls the printing rate.
        """
        while not rospy.is_shutdown():
            if self.last_joy_msg is not None:
                current_time = time.time()
                
                # Print only at the defined rate to keep the terminal readable
                if current_time - self.last_print_time >= self.print_interval:
                    self.print_joy_state(self.last_joy_msg)
                    self.last_print_time = current_time
            
            # Sleep to maintain the desired loop rate
            self.publish_rate.sleep()

    def print_joy_state(self, joy_msg):
        """
        Formats and prints the joystick state.
        DS4 Axis Mapping (Typical, may vary):
        Axes[0]: Left Stick (Left/Right) [-1.0 to 1.0]
        Axes[1]: Left Stick (Up/Down) [-1.0 to 1.0]
        Axes[2]: L2 Trigger (Unpressed 1.0, Fully Pressed -1.0)
        Axes[3]: Right Stick (Left/Right) [-1.0 to 1.0]
        Axes[4]: Right Stick (Up/Down) [-1.0 to 1.0]
        Axes[5]: R2 Trigger (Unpressed 1.0, Fully Pressed -1.0)
        Axes[6]: D-Pad Left/Right (-1.0=Left, 1.0=Right, 0.0=Center)
        Axes[7]: D-Pad Up/Down (-1.0=Down, 1.0=Up, 0.0=Center)
        
        DS4 Button Mapping (Typical, may vary):
        Buttons[0]: Square
        Buttons[1]: X
        Buttons[2]: Circle
        Buttons[3]: Triangle
        Buttons[4]: L1
        Buttons[5]: R1
        Buttons[6]: L2 (Button click, not analog)
        Buttons[7]: R2 (Button click, not analog)
        Buttons[8]: Share
        Buttons[9]: Options
        Buttons[10]: PS Button
        Buttons[11]: Left Stick Click (L3)
        Buttons[12]: Right Stick Click (R3)
        """
        
        # Format Axes
        axes_str = ", ".join(f"A{i}: {round(v, 3): <6}" for i, v in enumerate(joy_msg.axes))
        
        # Format Buttons (only print pressed buttons for simplicity)
        pressed_buttons = [f"B{i}" for i, b in enumerate(joy_msg.buttons) if b == 1]
        buttons_str = f"Pressed: [{', '.join(pressed_buttons)}]"
        
        # Clear the line and print the new state
        print(f"\rAxes: [{axes_str}] | Buttons: {buttons_str} \t\t\t\t", end='')


if __name__ == '__main__':
    try:
        listener = DS4Listener()
        listener.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Print a newline when the node shuts down
        print("\nDS4 Listener node shut down.")