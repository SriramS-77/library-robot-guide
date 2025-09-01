#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import tf

# Global variable to store the path
path_data = None

def set_initial_pose(x, y, yaw):
    """
    Publishes the initial pose of the robot.
    """
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
    msg.pose.covariance = [0.01]*36
    pub.publish(msg)
    rospy.loginfo(f"Initial pose set to x={x}, y={y}, yaw={yaw}")

def send_goal(x, y, yaw):
    """
    Sends a goal pose to the move_base server.
    """
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(1)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = x
    msg.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    pub.publish(msg)
    rospy.loginfo(f"Goal pose set to x={x}, y={y}, yaw={yaw}")

def path_callback(msg):
    """
    Callback function to receive the path from the global planner.
    """
    global path_data
    path_data = msg
    rospy.loginfo("Path received with %d poses.", len(path_data.poses))
    print(path_data.poses[:10])
    # Once we have the path, we can stop the node
    rospy.signal_shutdown("Path received")

if __name__ == "__main__":
    rospy.init_node('set_pose_and_get_path')

    # --- Set Your Desired Poses Here ---
    # Initial Pose (in meters and radians)
    initial_x, initial_y, initial_yaw = 4.0, 3.0, 1.57 #1.0, 1.0, 0.0

    # Goal Pose (in meters and radians)
    goal_x, goal_y, goal_yaw = 4.0, 5.0, 1.57
    # ------------------------------------

    # Subscriber for the path
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback)

    set_initial_pose(initial_x, initial_y, initial_yaw)
    send_goal(goal_x, goal_y, goal_yaw)

    rospy.spin()

    if path_data:
        rospy.loginfo("Path acquisition successful.")
    else:
        rospy.logwarn("No path was received.")
        