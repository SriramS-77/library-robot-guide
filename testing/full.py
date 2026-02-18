#!/usr/bin/env python3

import time
import math

# Mock GPIO for testing on a non-RPi machine.
# On the RPi, comment this block out and install RPi.GPIO.
try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    print("----------------------------------------------------------------")
    print("WARNING: RPi.GPIO library not found or not running on RPi.")
    print("         Using a mock GPIO library for testing purposes.")
    print("----------------------------------------------------------------")
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        IN = "IN"
        HIGH = 1
        LOW = 0
        def setmode(self, mode): print(f"GPIO.setmode({mode})")
        def setup(self, pin, mode): print(f"GPIO.setup({pin}, {mode})")
        def output(self, pin, value): pass # print(f"GPIO.output({pin}, {value})")
        def input(self, pin): return self.LOW
        def cleanup(self): print("GPIO.cleanup()")
    GPIO = MockGPIO()


# ROS Imports
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf.transformations

# ==============================================================================
# CONFIGURATION CLASS
# ==============================================================================
class RobotConfig:
    """
    Holds all static configuration for the robot.
    !! YOU MUST TUNE THESE VALUES FOR YOUR ROBOT !!
    """
    # --- Physical Properties ---
    # Diameter of the robot's wheels in meters
    WHEEL_DIAMETER_M = 0.065
    # Distance between the center of the left and right wheels in meters
    WHEEL_BASE_M = 0.15
    WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * math.pi

    # --- Motor Speed Calibration ---
    # The robot's maximum linear speed in meters per second.
    # Calibrate this by telling the robot to move forward for 10s and measuring distance.
    # Speed = Distance / 10s
    MAX_SPEED_MPS = 0.25

    # --- GPIO Pin Configuration (using BCM numbering) ---
    MOTOR_PINS = {
        'left_forward': 23,
        'left_backward': 22,
        'right_forward': 27,
        'right_backward': 17,
    }
    ULTRASOUND_PINS = {
        'front': {'trig': 13, 'echo': 19},
        'left': {'trig': 26, 'echo': 21},
        'right': {'trig': 5, 'echo': 6},
    }

    # --- ROS Configuration ---
    ROS_NODE_NAME = "rpi_robot_controller"
    ROS_PUB_RATE_HZ = 1.0  # Publication frequency for odom and sensor data
    ROS_ODOM_TOPIC = "/odom"
    ROS_FRONT_SCAN_TOPIC = "/scan/ultrasound/front"
    ROS_LEFT_SCAN_TOPIC = "/scan/ultrasound/left"
    ROS_RIGHT_SCAN_TOPIC = "/scan/ultrasound/right"
    ODOM_FRAME_ID = "odom"
    BASE_LINK_FRAME_ID = "base_link"


# ==============================================================================
# HARDWARE ABSTRACTION CLASSES
# ==============================================================================
class MotorController:
    """Handles low-level control of the robot's motors."""
    def __init__(self, pins):
        self.pins = pins
        GPIO.setmode(GPIO.BCM)
        for pin in self.pins.values():
            GPIO.setup(pin, GPIO.OUT)
        self.stop()

    def forward(self):
        GPIO.output(self.pins['left_forward'], GPIO.HIGH)
        GPIO.output(self.pins['left_backward'], GPIO.LOW)
        GPIO.output(self.pins['right_forward'], GPIO.HIGH)
        GPIO.output(self.pins['right_backward'], GPIO.LOW)

    def backward(self):
        GPIO.output(self.pins['left_forward'], GPIO.LOW)
        GPIO.output(self.pins['left_backward'], GPIO.HIGH)
        GPIO.output(self.pins['right_forward'], GPIO.LOW)
        GPIO.output(self.pins['right_backward'], GPIO.HIGH)

    def turn_left(self): # Rotate counter-clockwise
        GPIO.output(self.pins['left_forward'], GPIO.LOW)
        GPIO.output(self.pins['left_backward'], GPIO.HIGH)
        GPIO.output(self.pins['right_forward'], GPIO.HIGH)
        GPIO.output(self.pins['right_backward'], GPIO.LOW)

    def turn_right(self): # Rotate clockwise
        GPIO.output(self.pins['left_forward'], GPIO.HIGH)
        GPIO.output(self.pins['left_backward'], GPIO.LOW)
        GPIO.output(self.pins['right_forward'], GPIO.LOW)
        GPIO.output(self.pins['right_backward'], GPIO.HIGH)

    def stop(self):
        for pin in self.pins.values():
            GPIO.output(pin, GPIO.LOW)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()

class UltrasoundSensor:
    """Handles reading distance from an HC-SR04 sensor."""
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.sound_speed_mps = 343.0  # Speed of sound in meters/second

        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(0.1) # Allow sensor to settle

    def get_distance(self):
        """Returns the measured distance in meters."""
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)

        pulse_start_time = time.time()
        pulse_end_time = time.time()
        
        timeout = pulse_start_time + 0.1 # 100ms timeout

        while GPIO.input(self.echo_pin) == 0 and pulse_start_time < timeout:
            pulse_start_time = time.time()

        while GPIO.input(self.echo_pin) == 1 and pulse_end_time < timeout:
            pulse_end_time = time.time()

        if pulse_end_time >= timeout:
            return float('inf') # Return infinity on timeout

        pulse_duration = pulse_end_time - pulse_start_time
        distance = (pulse_duration * self.sound_speed_mps) / 2
        return distance

    def get_range_message(self, frame_id):
        """Creates a ROS sensor_msgs/Range message."""
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = math.radians(15) # Approx. 15 degrees
        range_msg.min_range = 0.02 # meters
        range_msg.max_range = 4.0  # meters
        range_msg.range = self.get_distance()
        return range_msg

# ==============================================================================
# LOGIC AND CONTROL CLASSES
# ==============================================================================
class OdometryCalculator:
    """
    Calculates and stores the robot's pose (x, y, theta) using dead reckoning.
    """
    def __init__(self, wheel_base_m):
        self.wheel_base = wheel_base_m
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # Heading in radians

    def update_from_movement(self, distance_m):
        """Update pose based on linear movement."""
        self.x += distance_m * math.cos(self.theta)
        self.y += distance_m * math.sin(self.theta)

    def update_from_rotation(self, angle_rad):
        """Update pose based on rotation."""
        self.theta += angle_rad
        # Normalize theta to be between -pi and pi
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def get_odometry_message(self, config: RobotConfig):
        """Creates a ROS nav_msgs/Odometry message."""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = config.ODOM_FRAME_ID
        odom.child_frame_id = config.BASE_LINK_FRAME_ID

        # Set the position
        odom.pose.pose.position = Point(self.x, self.y, 0.0)

        # Convert theta (Euler angle) to a quaternion
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity (optional, can be left as zero if not measured)
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        return odom

class RobotController:
    """
    Main class to control the robot, manage state, and interact with ROS.
    """
    def __init__(self):
        self.config = RobotConfig()
        
        # Initialize ROS Node
        rospy.init_node(self.config.ROS_NODE_NAME, anonymous=True)
        self.rate = rospy.Rate(self.config.ROS_PUB_RATE_HZ)
        self.last_pub_time = rospy.Time.now()

        # Initialize Hardware and Logic
        self.motors = MotorController(self.config.MOTOR_PINS)
        self.odometry = OdometryCalculator(self.config.WHEEL_BASE_M)
        self.sensors = {
            'front': UltrasoundSensor(**self.config.ULTRASOUND_PINS['front']),
            'left': UltrasoundSensor(**self.config.ULTRASOUND_PINS['left']),
            'right': UltrasoundSensor(**self.config.ULTRASOUND_PINS['right']),
        }

        # Initialize ROS Publishers
        self.odom_pub = rospy.Publisher(self.config.ROS_ODOM_TOPIC, Odometry, queue_size=10)
        self.scan_pubs = {
            'front': rospy.Publisher(self.config.ROS_FRONT_SCAN_TOPIC, Range, queue_size=5),
            'left': rospy.Publisher(self.config.ROS_LEFT_SCAN_TOPIC, Range, queue_size=5),
            'right': rospy.Publisher(self.config.ROS_RIGHT_SCAN_TOPIC, Range, queue_size=5),
        }
        
        print("Robot Controller initialized.")

    def publish_all_data(self):
        """Publish odometry and sensor data to their respective ROS topics."""
        # Odometry
        self.odom_pub.publish(self.odometry.get_odometry_message(self.config))
        # Ultrasound Sensors
        self.scan_pubs['front'].publish(self.sensors['front'].get_range_message("ultrasound_front_link"))
        self.scan_pubs['left'].publish(self.sensors['left'].get_range_message("ultrasound_left_link"))
        self.scan_pubs['right'].publish(self.sensors['right'].get_range_message("ultrasound_right_link"))
        self.last_pub_time = rospy.Time.now()
        rospy.loginfo(f"Published data. Pose: x={self.odometry.x:.2f}, y={self.odometry.y:.2f}, th={math.degrees(self.odometry.theta):.1f}°")

    def _execute_movement(self, motor_action, duration, odometry_update_func, odometry_update_val):
        """
        Private helper to run motors for a duration while periodically publishing data.
        """
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(duration)
        
        motor_action()

        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            if (rospy.Time.now() - self.last_pub_time).to_sec() >= (1.0 / self.config.ROS_PUB_RATE_HZ):
                self.publish_all_data()
            time.sleep(0.01) # Small sleep to prevent busy-waiting
        
        self.motors.stop()
        
        # After movement is complete, perform the final odometry update
        odometry_update_func(odometry_update_val)
        # Publish one final time to reflect the new state
        self.publish_all_data()

    def move_forward(self, distance_m: float):
        """Moves the robot forward by a specified distance."""
        if distance_m == 0: return
        rospy.loginfo(f"Moving forward by {distance_m:.2f} meters.")
        duration = abs(distance_m) / self.config.MAX_SPEED_MPS
        action = self.motors.forward if distance_m > 0 else self.motors.backward
        self._execute_movement(action, duration, self.odometry.update_from_movement, distance_m)

    def rotate_by_angle(self, angle_deg: float):
        """Rotates the robot by a specified angle in degrees (positive=clockwise)."""
        if angle_deg == 0: return
        rospy.loginfo(f"Rotating by {angle_deg:.1f} degrees.")
        
        angle_rad = math.radians(angle_deg)
        # Arc length that a wheel travels during rotation
        arc_length = abs(angle_rad) * (self.config.WHEEL_BASE_M / 2.0)
        duration = arc_length / self.config.MAX_SPEED_MPS
        
        action = self.motors.turn_right if angle_deg > 0 else self.motors.turn_left
        self._execute_movement(action, duration, self.odometry.update_from_rotation, angle_rad)

    def navigate_to_waypoint(self, target_x: float, target_y: float):
        """Calculates rotation and movement needed to reach a waypoint and executes it."""
        rospy.loginfo(f"--- Navigating to waypoint ({target_x:.2f}, {target_y:.2f}) ---")
        
        # 1. Calculate required rotation
        dx = target_x - self.odometry.x
        dy = target_y - self.odometry.y
        target_angle_rad = math.atan2(dy, dx)
        
        angle_to_rotate_rad = target_angle_rad - self.odometry.theta
        # Normalize to shortest rotation
        angle_to_rotate_rad = (angle_to_rotate_rad + math.pi) % (2 * math.pi) - math.pi
        
        # 2. Perform rotation
        self.rotate_by_angle(math.degrees(angle_to_rotate_rad))
        time.sleep(0.5) # Pause briefly after rotating

        # 3. Calculate required distance
        distance_to_move = math.sqrt(dx**2 + dy**2)
        
        # 4. Perform movement
        self.move_forward(distance_to_move)
        time.sleep(0.5) # Pause briefly after moving

    def run_path_navigation(self, path: list):
        """Executes a sequence of waypoints."""
        rospy.loginfo("Starting path navigation...")
        for i, waypoint in enumerate(path):
            if rospy.is_shutdown():
                break
            rospy.loginfo(f"Processing waypoint {i+1}/{len(path)}: {waypoint}")
            self.navigate_to_waypoint(waypoint[0], waypoint[1])
        
        rospy.loginfo("Path navigation complete.")

    def shutdown(self):
        """Gracefully shuts down the robot and cleans up GPIO."""
        print("\nShutting down robot controller.")
        self.motors.cleanup()


# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
if __name__ == '__main__':
    robot = None
    try:
        robot = RobotController()
        
        # Define the path for the robot to follow.
        # This is a list of (x, y) coordinates in meters.
        # Example: Move in a 1-meter square.
        navigation_path = [
            (1.0, 0.0),    # Go 1m forward
            (1.0, 1.0),    # Turn left, go 1m
            (0.0, 1.0),    # Turn left, go 1m
            (0.0, 0.0),    # Turn left, go 1m
        ]
        
        # Give ROS some time to initialize publishers/subscribers
        time.sleep(1)

        # Run the navigation task
        robot.run_path_navigation(navigation_path)

    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except KeyboardInterrupt:
        print("Program interrupted by user (Ctrl+C).")
    finally:
        if robot:
            robot.shutdown()