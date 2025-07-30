#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from rover_control.motor_driver import MotorDriver
import json
import os

def motor_callback(msg):
    command = msg.data.lower()
    rospy.loginfo(f"[motor_node] Received command: {command}")

    if command == "forward":
        driver.move_forward(config.get("forward_duration", 2.0))
    elif command == "backward":
        driver.move_backward(config.get("forward_duration", 2.0))
    elif command == "left":
        driver.turn_left(config.get("turn_duration", 1.0))
    elif command == "right":
        driver.turn_right(config.get("turn_duration", 1.0))
    elif command == "stop":
        driver.stop()
    else:
        rospy.logwarn(f"[motor_node] Unknown command: {command}")

def motor_node():
    global driver, config

    rospy.init_node('motor_node')
    rospy.loginfo("[motor_node] Node initialized")

    # Load config
    config_path = os.path.join(os.path.dirname(__file__), '..', 'rover_control', 'config.json')
    with open(config_path, 'r') as f:
        config = json.load(f)

    left_pins = config['left_motor_pins']
    right_pins = config['right_motor_pins']

    # Initialize motor driver
    driver = MotorDriver(left_motor_pins=left_pins, right_motor_pins=right_pins)
    rospy.loginfo("[motor_node] MotorDriver initialized")

    # Subscribe to motor command topic
    rospy.Subscriber('motor_commands', String, motor_callback)
    rospy.loginfo("[motor_node] Subscribed to /motor_commands")

    rospy.spin()

if __name__ == '__main__':
    motor_node()
