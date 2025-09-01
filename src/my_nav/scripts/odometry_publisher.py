#!/usr/bin/env python3

"""
Simple odometry publisher that integrates cmd_vel and publishes:
 - nav_msgs/Odometry on /odom
 - tf transform odom -> base_link

Use for simulation or when you want a simple odom->base_link publisher.
"""

import rospy
import math
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
from std_msgs.msg import Header
import threading
import time

class OdomPublisher:
    def __init__(self):
        rospy.init_node('simple_odometry_publisher', anonymous=False)

        # Parameters
        self.rate_hz = rospy.get_param('~rate', 50)
        self.frame_odom = rospy.get_param('~frame_odom', 'odom')
        self.frame_base = rospy.get_param('~frame_base', 'base_link')

        # Integration state
        self.x = float(rospy.get_param('~init_x', 0.0))
        self.y = float(rospy.get_param('~init_y', 0.0))
        self.yaw = float(rospy.get_param('~init_yaw', 0.0))

        self.vx = 0.0
        self.vy = 0.0  # if differential drive ignore, but keep for generality
        self.vth = 0.0

        self.lock = threading.RLock()

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscriber to /cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_cb, queue_size=10)

        self.last_time = rospy.Time.now()

    def cmdvel_cb(self, msg):
        with self.lock:
            # Accept linear.x and angular.z (typical differential drive)
            self.vx = msg.linear.x
            self.vy = msg.linear.y if hasattr(msg.linear, 'y') else 0.0
            self.vth = msg.angular.z

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / float(self.rate_hz)

            with self.lock:
                # integrate pose (simple Euler integration)
                # Assuming velocities are in the robot (base_link) frame; convert to odom frame motion
                # For differential drive, vy is usually zero.
                dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
                dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
                dyaw = self.vth * dt

                self.x += dx
                self.y += dy
                self.yaw += dyaw
                self.yaw = (self.yaw + math.pi) % (2.0*math.pi) - math.pi

                # Build quaternion
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)

                # Publish transform odom -> base_link
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = self.frame_odom
                t.child_frame_id = self.frame_base
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_broadcaster.sendTransform(t)

                # Publish odom message
                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = self.frame_odom
                odom.child_frame_id = self.frame_base
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = Quaternion(*q)

                odom.twist.twist.linear.x = self.vx
                odom.twist.twist.linear.y = self.vy
                odom.twist.twist.angular.z = self.vth

                # Optional: small covariances
                odom.pose.covariance = [0.0001] * 36
                odom.twist.covariance = [0.0001] * 36

                self.odom_pub.publish(odom)
                self.last_time = now

            r.sleep()

if __name__ == '__main__':
    node = OdomPublisher()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
