#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped

def publish_circular_trajectory():
    pub = rospy.Publisher('/demo/pose_final', PoseStamped, queue_size=10)
    rospy.init_node('circular_pose_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    center_x = 0.4
    center_y = 0.0
    center_z = 0.5
    radius = 0.1
    angular_velocity = 2 * math.pi / 10.0  # One full circle in 10 seconds

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        t = current_time - start_time
        angle = angular_velocity * t

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'fr3_link0'

        pose_msg.pose.position.x = center_x + radius * math.cos(angle)
        pose_msg.pose.position.y = center_y + radius * math.sin(angle)
        pose_msg.pose.position.z = center_z

        # Costante orientazione (quaternion) - x = 1.0, y = 0.0, z = 0.0, w = 0.0
        pose_msg.pose.orientation.x = 1.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0

        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_circular_trajectory()
    except rospy.ROSInterruptException:
        pass
