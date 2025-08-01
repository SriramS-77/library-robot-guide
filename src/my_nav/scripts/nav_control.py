#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

def set_initial_pose(x, y, yaw):
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    # small covariance: high confidence
    msg.pose.covariance = [0.01]*36
    pub.publish(msg)
    rospy.loginfo(f"Initial pose set to x={x}, y={y}, yaw={yaw}")

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    client.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    client.wait_for_result()
    rospy.loginfo("Result: %s", client.get_state())

if __name__ == "__main__":
    rospy.init_node('nav_control')
    # Replace with your desired start & goal:
    set_initial_pose(1.0, 1.0, 0.0)
    send_goal(4.0, 3.0, 1.57)
