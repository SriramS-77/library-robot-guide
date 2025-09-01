#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

def publish_odometry():
    """
    Publishes simulated odometry data.
    """
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rospy.init_node('simulate_odometry', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    x, y, yaw = 0.0, 0.0, 0.0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Update pose (in a real simulation, this would be based on velocity)
        # For now, let's keep it static at the initial pose from the other script
        x = 1.0
        y = 1.0
        yaw = 0.0

        odom.header.stamp = current_time
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set a small covariance
        odom.pose.covariance = [0.1] * 36

        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odometry()
    except rospy.ROSInterruptException:
        pass
        