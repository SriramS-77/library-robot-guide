#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import time

class PathFollowerSimulator:
    """
    A class to set a pose and goal, receive a path, and simulate
    odometry by following that path.
    """
    def __init__(self):
        # --- Configuration ---
        # Poses
        self.initial_pose = (2.0, 2.0, 0.0)  # (x, y, yaw)
        self.goal_pose = (4.0, 3.0, 1.57)    # (x, y, yaw)

        # Simulation parameters
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.goal_tolerance = 0.15 # meters
        self.loop_rate = 2         # Hz (recommended)
        self.cmd_vel_timeout = 0.5  # seconds: if no cmd_vel in this time, treat as zero

        # --- Internal State ---
        self.current_path = None
        self.path_received = False
        self.current_path_segment = 0
        self.current_pose = {
            "x": self.initial_pose[0],
            "y": self.initial_pose[1],
            "yaw": self.initial_pose[2]
        }

        # Latest cmd_vel
        self.last_cmd = Twist()
        self.last_cmd_time = None

        # --- ROS Setup ---
        rospy.init_node('path_follower_simulator')

        # Publishers
        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Subscriber
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)


    def cmd_vel_callback(self, msg):
        """Store latest cmd_vel and timestamp."""
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()
        print("Velocity Received...", msg)


    def path_callback(self, msg):
        """Stores the received path and resets the path segment index."""
        if msg.poses:
            self.current_path = msg.poses
            self.current_path_segment = 0
            if not self.path_received:
                rospy.loginfo("Initial path received with %d waypoints. Starting simulation.", len(self.current_path))
                self.path_received = True
                # print(self.current_path[:10])
            else:
                rospy.loginfo(f"Planner replanned. Following new path of length: {len(self.current_path)}")

    def set_initial_pose(self):
        """Publishes the initial pose of the robot."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = self.initial_pose[0]
        msg.pose.pose.position.y = self.initial_pose[1]
        q = tf.transformations.quaternion_from_euler(0, 0, self.initial_pose[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = [0.01] * 36
        self.initial_pose_pub.publish(msg)
        rospy.loginfo(f"Initial pose published: {self.initial_pose}")

    def send_goal(self):
        """Sends the goal pose to the move_base server."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.goal_pose[0]
        msg.pose.position.y = self.goal_pose[1]
        q = tf.transformations.quaternion_from_euler(0, 0, self.goal_pose[2])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.goal_pub.publish(msg)
        rospy.loginfo(f"Goal pose published: {self.goal_pose}")

    def run_simulation(self):
        """The main simulation loop."""
        rate = rospy.Rate(self.loop_rate)
        
        # Give a moment for publishers to connect
        rospy.sleep(1.0)
        self.set_initial_pose()
        rospy.sleep(1.0)
        self.send_goal()


        prev_time = rospy.Time.now()
        rospy.loginfo("Starting simulation loop at %d Hz", self.loop_rate)

        while not rospy.is_shutdown():
            if not self.path_received:
                rospy.loginfo_once("Waiting for path from global planner...")
                rate.sleep()
                continue

            now = rospy.Time.now()
            dt = (now - prev_time).to_sec() 
            if dt <= 0.0:
                dt = 1.0 / float(self.loop_rate)

            # Check if goal is reached
            goal_dist = math.sqrt((self.goal_pose[0] - self.current_pose["x"])**2 + \
                                (self.goal_pose[1] - self.current_pose["y"])**2)
            if goal_dist < self.goal_tolerance:
                rospy.loginfo("Goal reached! Shutting down.")
                break

            # Determine velocities to use from cmd_vel (with timeout)
            use_linear = 0.0
            use_angular = 0.0
            if self.last_cmd_time is not None:
                age = (now - self.last_cmd_time).to_sec()
                if age <= self.cmd_vel_timeout:
                    # Use the latest cmd_vel (clamped to safety limits)
                    use_linear = max(min(self.last_cmd.linear.x, self.max_linear_vel), -self.max_linear_vel)
                    use_angular = max(min(self.last_cmd.angular.z, self.max_angular_vel), -self.max_angular_vel)
                else:
                    # timeout: no recent cmd_vel -> stop
                    use_linear = 0.0
                    use_angular = 0.0
            else:
                # no cmd_vel received yet -> zero velocities
                use_linear = 0.0
                use_angular = 0.0

            # Update pose by integrating velocities
            self.current_pose["yaw"] += use_angular * dt
            # normalize yaw into -pi..pi (optional)
            self.current_pose["yaw"] = (self.current_pose["yaw"] + math.pi) % (2 * math.pi) - math.pi

            # integrate (assume differential drive, x-forward)
            self.current_pose["x"] += self.last_cmd.linear.x * 0.5 #* math.cos(self.current_pose["yaw"]) * dt
            self.current_pose["y"] += self.last_cmd.linear.y * 0.5 #* math.sin(self.current_pose["yaw"]) * dt

            # Publish odometry
            self.publish_odometry(use_linear, use_angular)

            prev_time = now
            rate.sleep()

            # # --- Path Following Logic ---
            # if self.current_path_segment >= len(self.current_path):
            #     # Reached end of path but not the goal, wait for replan
            #     linear_vel, angular_vel = 0.0, 0.0
            # else:
            #     # Get the next waypoint
            #     target_pose = self.current_path[self.current_path_segment].pose
            #     target_x = target_pose.position.x
            #     target_y = target_pose.position.y

            #     # Calculate distance and angle to the next waypoint
            #     dx = target_x - self.current_pose["x"]
            #     dy = target_y - self.current_pose["y"]
            #     distance_to_target = math.sqrt(dx**2 + dy**2)
            #     angle_to_target = math.atan2(dy, dx)

            #     # If close enough to waypoint, move to the next one
            #     if distance_to_target < 0.2:
            #         self.current_path_segment += 1
            #         print("Moving to next wp in path...")
            #         if self.current_path_segment >= len(self.current_path):
            #             rospy.loginfo("Reached end of current path segments.")
            #             continue

            #     # --- Simple P-Controller for Velocity ---
            #     # Angular velocity
            #     angle_error = angle_to_target - self.current_pose["yaw"]
            #     # Normalize angle error to [-pi, pi]
            #     angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            #     angular_vel = 2.0 * angle_error # Proportional gain
            #     angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

            #     # Linear velocity
            #     linear_vel = 0.5 * distance_to_target # Proportional gain
            #     # Slow down if turning sharply
            #     if abs(angle_error) > math.pi / 4:
            #         linear_vel = self.max_linear_vel * 0.5
            #     linear_vel = min(linear_vel, self.max_linear_vel)


            # # --- Update and Publish Odometry ---
            # dt = 1.0 / self.loop_rate
            # self.current_pose["yaw"] += angular_vel * dt
            # self.current_pose["x"] += linear_vel * math.cos(self.current_pose["yaw"]) * dt
            # self.current_pose["y"] += linear_vel * math.sin(self.current_pose["yaw"]) * dt

            # self.publish_odometry(linear_vel, angular_vel)

            # rate.sleep()

    def publish_odometry(self, linear_vel, angular_vel):
        """Creates and publishes an Odometry message."""
        odom = Odometry()
        current_time = rospy.Time.now()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.current_pose["x"]
        odom.pose.pose.position.y = self.current_pose["y"]
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, self.current_pose["yaw"])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # Set a small covariance
        odom.pose.covariance = [0.1] * 36
        odom.twist.covariance = [0.1] * 36

        self.odom_pub.publish(odom)

        rospy.loginfo(f"Publishing odometry: pos=({self.current_pose['x']:.2f},{self.current_pose['y']:.2f}), yaw={self.current_pose['yaw']:.2f}")
        # print(f"Publishing odometry: {odom.pose.pose}")

if __name__ == '__main__':
    try:
        simulator = PathFollowerSimulator()
        simulator.run_simulation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulation interrupted.")
