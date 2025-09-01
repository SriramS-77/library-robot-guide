#!/usr/bin/env python3

import time
import math
import rospy
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor

class UltrasoundSensor:
    """Handles reading distance from an HC-SR04 sensor using gpiozero."""
    def __init__(self, trig_pin, echo_pin):
        # gpiozero DistanceSensor uses echo and trigger pins (note the order!)
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=4.0)
        self.sound_speed_mps = 343.0  # for consistency, though not needed here
        time.sleep(0.1)  # allow sensor to settle


    def get_distance(self):
        """Returns the measured distance in meters."""
        # gpiozero DistanceSensor reports normalized distance (0.0 → max_distance)
        # or you can directly access .distance in meters if max_distance is set
        distance = self.sensor.distance
        return distance

    def close(self):
        """Release GPIO resources."""
        self.sensor.close()

class SonarPublisher:
    def __init__(self):
        rospy.init_node('sonar_publisher', anonymous=True)

        # --- GPIO Pin Configuration (using BCM numbering) ---
        # These should match your physical wiring
        self.ULTRASOUND_PINS = {
            'front': {'trig_pin': 13, 'echo_pin': 19},
            'left': {'trig_pin': 26, 'echo_pin': 21},
            'right': {'trig_pin': 5, 'echo_pin': 6},
        }

        # --- Sensor Objects ---
        self.sensors = {
            name: UltrasoundSensor(**pins)
            for name, pins in self.ULTRASOUND_PINS.items()
        }

        # --- ROS Publishers ---
        self.pubs = {
            'front': rospy.Publisher('/ultrasound/front', Range, queue_size=5),
            'left': rospy.Publisher('/ultrasound/left', Range, queue_size=5),
            'right': rospy.Publisher('/ultrasound/right', Range, queue_size=5),
        }

        # --- ROS Rate ---
        self.rate = rospy.Rate(10) # 10 Hz

        rospy.loginfo("Sonar publisher node started.")

    def create_range_message(self, frame_id, distance):
        """Creates a ROS sensor_msgs/Range message."""
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = math.radians(20) # Approx. 20 degrees
        range_msg.min_range = 0.02 # meters
        range_msg.max_range = 4.0  # meters
        range_msg.range = distance
        return range_msg

    def run(self):
        """Main loop to read from sensors and publish data."""
        while not rospy.is_shutdown():
            # Read and publish front sensor
            dist_front = self.sensors['front'].get_distance()
            self.pubs['front'].publish(self.create_range_message('ultrasound_front_link', dist_front))

            # Read and publish left sensor
            dist_left = self.sensors['left'].get_distance()
            self.pubs['left'].publish(self.create_range_message('ultrasound_left_link', dist_left))

            # Read and publish right sensor
            dist_right = self.sensors['right'].get_distance()
            self.pubs['right'].publish(self.create_range_message('ultrasound_right_link', dist_right))

            # rospy.loginfo(f"Sonar readings: {dist_left}, {dist_front}, {dist_right}")

            self.rate.sleep()

    def shutdown(self):
        """Cleans up GPIO resources."""
        print("Shutting down sonar publisher.")
        for sensor in self.sensors.values():
            if sensor:
                sensor.close()

if __name__ == '__main__':
    try:
        publisher = SonarPublisher()
        rospy.on_shutdown(publisher.shutdown)
        publisher.run()
    except rospy.ROSInterruptException:
        pass

