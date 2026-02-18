import RPi.GPIO as GPIO

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

    # def get_range_message(self, frame_id):
    #     """Creates a ROS sensor_msgs/Range message."""
    #     range_msg = Range()
    #     range_msg.header.stamp = rospy.Time.now()
    #     range_msg.header.frame_id = frame_id
    #     range_msg.radiation_type = Range.ULTRASOUND
    #     range_msg.field_of_view = math.radians(15) # Approx. 15 degrees
    #     range_msg.min_range = 0.02 # meters
    #     range_msg.max_range = 4.0  # meters
    #     range_msg.range = self.get_distance()
    #     return range_msg

sensor = UltrasoundSensor(trig_pin=13, echo_pin=19)

# sensor = UltrasoundSensor(trig_pin=26, echo_pin=21)

# sensor = UltrasoundSensor(trig_pin=5, echo_pin=6)

print(sensor.get_distance())
