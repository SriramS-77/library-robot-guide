#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Range, LaserScan

class SonarToScan:
    def __init__(self):
        rospy.init_node('sonar_to_scan', anonymous=True)

        # --- Subscribers ---
        rospy.Subscriber('/ultrasound/front', Range, self.front_cb)
        rospy.Subscriber('/ultrasound/left', Range, self.left_cb)
        rospy.Subscriber('/ultrasound/right', Range, self.right_cb)

        # --- Publisher ---
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)

        # --- Latest Readings ---
        self.latest_ranges = {'front': None, 'left': None, 'right': None}

        # --- LaserScan Configuration ---
        self.scan = LaserScan()
        self.scan.header.frame_id = 'base_link'
        self.scan.angle_min = -math.pi / 2  # -90 degrees
        self.scan.angle_max = math.pi / 2   # +90 degrees
        self.scan.angle_increment = math.pi / 2 # math.pi / 180.0 # 1 degree resolution
        self.scan.time_increment = 0.0
        self.scan.scan_time = 0.1 # Corresponds to 10Hz
        self.scan.range_min = 0.02
        self.scan.range_max = 4.0

        num_readings = int((self.scan.angle_max - self.scan.angle_min) / self.scan.angle_increment) + 1
        self.scan.ranges = [float('inf')] * num_readings

        # --- ROS Rate ---
        self.rate = rospy.Rate(10) # 10 Hz

        rospy.loginfo("Sonar to LaserScan conversion node started.")

    def front_cb(self, msg):
        self.latest_ranges['front'] = msg.range

    def left_cb(self, msg):
        self.latest_ranges['left'] = msg.range

    def right_cb(self, msg):
        self.latest_ranges['right'] = msg.range

    def run(self):
        """Main loop to construct and publish the LaserScan message."""
        while not rospy.is_shutdown():
            num_readings = len(self.scan.ranges)
            
            # Create a fresh list of ranges filled with infinity
            current_ranges = [float('inf')] * num_readings

            # --- Map sonar readings to laser scan angles ---
            # Right sensor (-90 degrees)
            if self.latest_ranges['right'] is not None:
                right_index = 2 #0
                current_ranges[right_index] = self.latest_ranges['right']

            # Front sensor (0 degrees)
            if self.latest_ranges['front'] is not None:
                front_index = num_readings // 2
                current_ranges[front_index] = self.latest_ranges['front']

            # Left sensor (+90 degrees)
            if self.latest_ranges['left'] is not None:
                left_index = 0 #num_readings - 1
                current_ranges[left_index] = self.latest_ranges['left']

            # --- Publish the final LaserScan message ---
            self.scan.header.stamp = rospy.Time.now()
            self.scan.ranges = current_ranges
            self.scan_pub.publish(self.scan)

            # rospy.loginfo(f"scan data: {current_ranges}, {self.scan.header.stamp}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        converter = SonarToScan()
        converter.run()
    except rospy.ROSInterruptException:
        pass
