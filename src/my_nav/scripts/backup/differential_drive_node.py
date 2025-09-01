#!/usr/bin/env python3
"""
differential_drive_node.py

- Subscribes to /cmd_vel (geometry_msgs/Twist) coming from move_base/DWA.
- Converts (linear.x, angular.z) -> left/right wheel linear velocities (m/s).
- Sends scaled speed commands to the motor controller (gpiozero Motor).
- Smooths velocities using acceleration limits.
- Implements watchdog: if no cmd_vel for `cmd_timeout` seconds, stops motors.
- Publishes /odom (nav_msgs/Odometry) and broadcasts tf: odom -> base_link.
- Optionally uses encoder feedback (plug in EncoderReader) to compute odometry.
"""

import rospy
import math
import threading
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf_conversions
import time

# Try to import gpiozero; allow a simulation fallback
try:
    from gpiozero import Motor as GpioMotor
    GPIOZERO_AVAILABLE = True
except Exception:
    GPIOZERO_AVAILABLE = False

# -------------------------
# Motor controller (modular)
# -------------------------
class MotorInterface:
    """Abstract motor interface: set wheel speeds in linear m/s."""
    def set_wheel_speeds(self, left_speed_m_s, right_speed_m_s):
        raise NotImplementedError()
    def stop(self):
        raise NotImplementedError()
    def cleanup(self):
        raise NotImplementedError()

class GpioMotorController(MotorInterface):
    """gpiozero Motor wrapper. Accepts wheel linear speeds (m/s) and maps them to -1..1 pwm."""
    def __init__(self, pins, max_wheel_speed):
        if not GPIOZERO_AVAILABLE:
            raise RuntimeError("gpiozero not available on this system")
        self.max_wheel_speed = float(max_wheel_speed)
        # create motors with (forward, backward)
        self.left_motor = GpioMotor(forward=pins['left_forward'], backward=pins['left_backward'])
        self.right_motor = GpioMotor(forward=pins['right_forward'], backward=pins['right_backward'])
        # last commanded for safe shutdown
        self._last_left_cmd = 0.0
        self._last_right_cmd = 0.0
        self.stop()

    def _apply_to_motor(self, motor, speed_norm):
        """speed_norm: -1..1. gpiozero accepts forward(value) / backward(value), value in 0..1"""
        if speed_norm >= 0:
            motor.forward(min(abs(speed_norm), 1.0))
        else:
            motor.backward(min(abs(speed_norm), 1.0))

    def set_wheel_speeds(self, left_speed_m_s, right_speed_m_s):
        # Map linear speeds to normalized motor values
        ln = left_speed_m_s / self.max_wheel_speed
        rn = right_speed_m_s / self.max_wheel_speed
        # clamp
        ln = max(min(ln, 1.0), -1.0)
        rn = max(min(rn, 1.0), -1.0)
        self._apply_to_motor(self.left_motor, ln)
        self._apply_to_motor(self.right_motor, rn)
        self._last_left_cmd = ln
        self._last_right_cmd = rn

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self._last_left_cmd = 0.0
        self._last_right_cmd = 0.0

    def cleanup(self):
        self.stop()
        # gpiozero auto cleanup on program exit

class DummyMotorController(MotorInterface):
    """For testing without hardware - just logs and stores commanded speeds."""
    def __init__(self, max_wheel_speed):
        self.max_wheel_speed = float(max_wheel_speed)
        self.left = 0.0
        self.right = 0.0

    def set_wheel_speeds(self, left_speed_m_s, right_speed_m_s):
        self.left = max(min(left_speed_m_s, self.max_wheel_speed), -self.max_wheel_speed)
        self.right = max(min(right_speed_m_s, self.max_wheel_speed), -self.max_wheel_speed)
        rospy.logdebug(f"[DummyMotor] left={self.left:.3f} m/s right={self.right:.3f} m/s")

    def stop(self):
        self.left = 0.0
        self.right = 0.0

    def cleanup(self):
        self.stop()

# -------------------------
# Encoder reader (optional)
# -------------------------
class EncoderReader:
    """
    Placeholder encoder interface. If you have wheel encoders, implement these methods
    to return measured wheel linear velocities (m/s). For now it returns commanded wheel speeds,
    so odometry is open-loop.
    """
    def __init__(self):
        # If using real encoders, initialize GPIO/interrupts here.
        pass

    def get_wheel_speeds(self):
        """
        Return tuple (vl, vr) in m/s. When not implemented, return None to indicate
        "no encoder available" and the node will use commanded wheel speeds.
        """
        return None

# -------------------------
# Differential drive node
# -------------------------
class DifferentialDriveNode:
    def __init__(self):
        rospy.init_node('differential_drive_node')

        # Robot configuration - moved inside __init__ after rospy.init_node()
        self.wheel_diameter_m = rospy.get_param('~wheel_diameter_m', 0.065)   # meters
        self.wheel_radius_m = self.wheel_diameter_m / 2.0
        self.wheel_base_m = rospy.get_param('~wheel_base_m', 0.15)          # distance between wheels (m)
        self.max_robot_speed = rospy.get_param('~max_robot_speed', 1.12)     # max linear robot speed (m/s)
        # Maximum wheel linear speed (m/s) - if both wheels at max produces MAX_ROBOT_SPEED
        # For simplicity assume MAX_ROBOT_SPEED corresponds to both wheels at 1.0 command.
        self.max_wheel_linear_speed = rospy.get_param('~max_wheel_speed', self.max_robot_speed)
        # Acceleration limits (m/s^2 for linear wheel speed)
        self.acc_lin = rospy.get_param('~acc_lim_lin', 0.5)     # m/s^2
        self.acc_ang = rospy.get_param('~acc_lim_ang', 1.0)     # rad/s^2
        self.control_rate = rospy.get_param('~control_rate', 20) # Hz
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)  # seconds, stop if no cmd_vel
        self.use_gpio = rospy.get_param('~use_gpio', True)       # set False for testing without hardware
        self.motor_pins = rospy.get_param('~motor_pins', {
            'left_forward': 23,
            'left_backward': 22,
            'right_forward': 27,
            'right_backward': 17,
        })

        # Params
        self.wheel_base = self.wheel_base_m
        self.wheel_radius = self.wheel_radius_m
        self.max_wheel_speed = self.max_wheel_linear_speed
        self.rate_hz = self.control_rate

        # Frames
        self.frame_odom = rospy.get_param('~frame_odom', 'odom')
        self.frame_base = rospy.get_param('~frame_base', 'base_link')

        # Motor controller
        if self.use_gpio and GPIOZERO_AVAILABLE:
            rospy.loginfo("Using gpiozero Motor controller")
            self.motor = GpioMotorController(self.motor_pins, self.max_wheel_speed)
        else:
            if self.use_gpio and not GPIOZERO_AVAILABLE:
                rospy.logwarn("gpiozero not available, falling back to DummyMotorController")
            self.motor = DummyMotorController(self.max_wheel_speed)

        # Encoder (optional)
        self.enc = EncoderReader()

        # Command state
        self.target_v = 0.0    # desired robot linear velocity (m/s)
        self.target_w = 0.0    # desired robot angular velocity (rad/s)
        self.last_cmd_time = rospy.Time.now()

        # Internal wheel command state (linear m/s) - FIXED: Initialize these properly
        self.cmd_left = 0.0
        self.cmd_right = 0.0

        # Odometry state
        self.x = rospy.get_param('~init_x', 0.0)
        self.y = rospy.get_param('~init_y', 0.0)
        self.yaw = rospy.get_param('~init_yaw', 0.0)

        # Publishers / TF
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscriber to cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb, queue_size=5)

        # Lock
        self.lock = threading.RLock()

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            # Only consider linear.x and angular.z
            self.target_v = float(msg.linear.x)
            self.target_w = float(msg.angular.z)
            self.last_cmd_time = rospy.Time.now()

    def _kinematics_from_vw(self, v, w):
        """
        Convert robot (v, w) to wheel linear speeds (vl, vr) [m/s]
        v = (vl + vr) / 2
        w = (vr - vl) / wheel_base
        => vl = v - w*(wheel_base/2)
           vr = v + w*(wheel_base/2)
        """
        half = 0.5 * self.wheel_base
        vl = v - w * half
        vr = v + w * half
        return vl, vr

    def _limit_acceleration(self, prev, target, dt, max_acc):
        """
        Limit acceleration for a scalar value (linear m/s).
        prev: previous value
        target: desired value
        dt: time step
        max_acc: max acceleration (m/s^2)
        """
        if dt <= 0.0:
            return target
        max_delta = max_acc * dt
        delta = target - prev
        if abs(delta) <= max_delta:
            return target
        return prev + math.copysign(max_delta, delta)

    def _publish_odometry(self, vl, vr, dt):
        """
        Compute robot pose from wheel linear speeds vl, vr (m/s) over dt seconds.
        Uses differential drive forward kinematics.
        """
        # robot linear and angular velocities
        v = 0.5 * (vl + vr)
        w = (vr - vl) / self.wheel_base

        # integrate pose (simple Euler)
        dx = v * math.cos(self.yaw) * dt
        dy = v * math.sin(self.yaw) * dt
        dyaw = w * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Publish TF odom -> base_link
        now = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
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

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*q)

        # Fill twist (in child frame)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Set sensible covariances (tune these for your robot)
        cov_pose = [0.0] * 36
        cov_twist = [0.0] * 36
        # Pose covariance: x, y, yaw
        cov_pose[0] = 0.02  # var x (m^2)
        cov_pose[7] = 0.02  # var y (m^2)
        cov_pose[35] = 0.05 # var yaw (rad^2)
        odom.pose.covariance = cov_pose
        # Twist covariance
        cov_twist[0] = 0.02
        cov_twist[35] = 0.05
        odom.twist.covariance = cov_twist

        self.odom_pub.publish(odom)

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        prev = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - prev).to_sec()
            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / float(self.rate_hz)
            prev = now

            with self.lock:
                # timeout -> stop
                time_since_cmd = (now - self.last_cmd_time).to_sec()
                if time_since_cmd > self.cmd_timeout:
                    tgt_v = 0.0
                    tgt_w = 0.0
                else:
                    tgt_v = max(min(self.target_v, self.max_robot_speed), -self.max_robot_speed)
                    tgt_w = self.target_w

                rospy.loginfo(f"Target: {tgt_v}, {tgt_w}...")

                # compute desired wheel speeds from v,w kinematics
                desired_left, desired_right = self._kinematics_from_vw(tgt_v, tgt_w)

                rospy.loginfo(f"Left: {desired_left}, Right: {desired_right}...")

                # acceleration limiting applied to each wheel (linear)
                self.cmd_left = self._limit_acceleration(self.cmd_left, desired_left, dt, self.acc_lin)
                self.cmd_right = self._limit_acceleration(self.cmd_right, desired_right, dt, self.acc_lin)

                # If encoders are available, you can override these with measured speeds:
                enc_speeds = self.enc.get_wheel_speeds()
                if enc_speeds is not None:
                    vl_meas, vr_meas = enc_speeds
                else:
                    vl_meas, vr_meas = self.cmd_left, self.cmd_right

                # Command motors (maps linear speeds to motor pwm)
                try:
                    self.motor.set_wheel_speeds(self.cmd_left, self.cmd_right)
                    rospy.loginfo(f"Set Speed Left: {self.cmd_left}, Right: {self.cmd_right}...")
                except Exception as e:
                    rospy.logerr(f"Motor command failed: {e}")

                # publish odometry using measured (or commanded) wheel speeds
                self._publish_odometry(vl_meas, vr_meas, dt)

            r.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down motor and node.")
        try:
            self.motor.stop()
            self.motor.cleanup()
        except Exception:
            pass

# -------------------------
# Main
# -------------------------
if __name__ == '__main__':
    node = DifferentialDriveNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass