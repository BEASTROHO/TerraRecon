#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion_node', anonymous=False)
        rospy.loginfo("[sensor_fusion_node] Node started")

        # Initialize sensor values
        self.sensor_data = {
            'ultrasonic': None,
            'ir': None,
            'camera': None
        }

        # Subscribers
        rospy.Subscriber('/ultrasonic_data', Float32, self._callback_ultrasonic)
        rospy.Subscriber('/ir_data', Float32, self._callback_ir)
        rospy.Subscriber('/camera_distance', Float32, self._callback_camera)

        # Publisher
        self.fused_pub = rospy.Publisher('/fused_sensor_data', Float32, queue_size=10)

        # Loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def _callback_ultrasonic(self, msg):
        self.sensor_data['ultrasonic'] = msg.data
        rospy.logdebug(f"[sensor_fusion_node] Ultrasonic: {msg.data}")

    def _callback_ir(self, msg):
        self.sensor_data['ir'] = msg.data
        rospy.logdebug(f"[sensor_fusion_node] IR: {msg.data}")

    def _callback_camera(self, msg):
        self.sensor_data['camera'] = msg.data
        rospy.logdebug(f"[sensor_fusion_node] Camera: {msg.data}")

    def fuse_and_publish(self):
        while not rospy.is_shutdown():
            valid_readings = [v for v in self.sensor_data.values() if v is not None]

            if valid_readings:
                # Conservative fusion: take minimum distance
                fused_value = min(valid_readings)
                rospy.loginfo(f"[sensor_fusion_node] Fused distance: {fused_value:.2f}")
                self.fused_pub.publish(Float32(data=fused_value))
            else:
                rospy.logwarn("[sensor_fusion_node] No valid sensor data available")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SensorFusionNode()
        node.fuse_and_publish()
    except rospy.ROSInterruptException:
        rospy.loginfo("[sensor_fusion_node] Shutdown requested")
