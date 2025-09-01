#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def odom_cb(msg):
    pos = msg.pose.pose.position
    rospy.loginfo(f"Current position: x={pos.x:.2f}, y={pos.y:.2f}")

def send_goal(x, y, yaw=0.0):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = rospy.sin(yaw/2)
    goal.target_pose.pose.orientation.w = rospy.cos(yaw/2)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(f"Goal result: {client.get_state()}")

if __name__ == '__main__':
    rospy.init_node('nav_client')
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, lambda msg: rospy.loginfo(
        f"AMCL pose: {msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}"))
    rospy.sleep(1)
    # Example: set goal via Python args or input
    send_goal(1.0, 0.5, yaw=0)
    rospy.spin()
