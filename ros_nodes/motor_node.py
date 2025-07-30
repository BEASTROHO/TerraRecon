#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def motor_callback(msg):
    rospy.loginfo(f"Received motor command: {msg.data}")
    # Call motor_driver methods here

def motor_node():
    rospy.init_node('motor_node')
    rospy.Subscriber('motor_commands', String, motor_callback)
    rospy.spin()

if __name__ == '__main__':
    motor_node()
