# ros_nodes/navigation_node.py

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class NavigationNode:
    """
    ROS node for autonomous navigation.
    Uses LiDAR data to avoid obstacles and publishes velocity commands.
    """

    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.rate = rospy.Rate(10)
        self.obstacle_detected = False

    def lidar_callback(self, data):
        # Check for obstacles within 0.5 meters in front
        front_ranges = data.ranges[len(data.ranges)//2 - 10 : len(data.ranges)//2 + 10]
        if any(r < 0.5 for r in front_ranges if r > 0):
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def navigate(self):
        move_cmd = Twist()
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected! Stopping.")
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.5  # Rotate to avoid
            else:
                rospy.loginfo("Path clear. Moving forward.")
                move_cmd.linear.x = 0.3
                move_cmd.angular.z = 0.0

            self.cmd_pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav_node = NavigationNode()
        nav_node.navigate()
    except rospy.ROSInterruptException:
        pass
