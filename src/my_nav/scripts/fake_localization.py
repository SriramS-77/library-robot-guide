#!/usr/bin/env python3

# import rospy
# import tf2_ros
# from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

# def on_initialpose(msg):
#     t = TransformStamped()
#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = 'map'
#     t.child_frame_id  = 'odom'
#     # Copy position
#     t.transform.translation.x = msg.pose.pose.position.x
#     t.transform.translation.y = msg.pose.pose.position.y
#     t.transform.translation.z = msg.pose.pose.position.z
#     # Copy orientation
#     q = msg.pose.pose.orientation
#     t.transform.rotation = q
#     tfb.sendTransform(t)

# if __name__ == '__main__':
#     rospy.init_node('fake_localization')
#     # tfb = tf2_ros.StaticTransformBroadcaster()  # or use TransformBroadcaster() for dynamic
#     tfb = tf2_ros.TransformBroadcaster()
#     rospy.Subscriber('initialpose', PoseWithCovarianceStamped, on_initialpose)
#     rospy.spin()



import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf.transformations import euler_from_quaternion

class FakeLocalization:
    def __init__(self):
        rospy.init_node('fake_localization', anonymous=True)
        self.latest = None
        # Whenever rviz publishes a new initialpose, we cache it:
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_cb)
        # TF broadcaster for dynamic transforms
        self.broadcaster = tf2_ros.TransformBroadcaster()
        # Timer at 10Hz to keep sending the latest map->odom
        rospy.Timer(rospy.Duration(0.1), self.send_tf)
        rospy.loginfo("fake_localization ready—waiting for /initialpose...")
        rospy.spin()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.latest = msg
        rospy.loginfo("Received new initialpose: x=%.2f y=%.2f yaw=%.2f",
                      msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      # extract yaw from quaternion
                      euler_from_quaternion([
                          msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w
                      ])[2])

    def send_tf(self, event):
        if not self.latest:
            return
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        p = self.latest.pose.pose.position
        q = self.latest.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        FakeLocalization()
    except rospy.ROSInterruptException:
        pass
