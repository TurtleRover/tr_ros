import rospy

from threading import Thread, Event


class Controller(object):
    def __init__(self, max_wheel_speed=6.0, input_timeout=0.2):
        self.power = [0, 0, 0, 0]  # RR, RL, FR, FL
        self.max_wheel_speed = max_wheel_speed
        self.check_input = input_timeout > 0.0
        self.input_timeout = rospy.Duration(input_timeout)

        if self.check_input:
            self.input_received = Event()
            self.last_update = rospy.get_rostime()
            self.check_input_thread = Thread(target=self.check_input_loop)
            self.check_input_thread.daemon = True
            self.check_input_thread.start()

    def check_input_loop(self):
        while not rospy.is_shutdown():
            self.input_received.wait()

            while rospy.get_rostime() < self.last_update + self.input_timeout:
                rospy.sleep(self.last_update - rospy.get_rostime() + self.input_timeout)

            rospy.logwarn(("No input received for more than a specified timeout! "
                           "Stopping the motors"))
            self.power = [0, 0, 0, 0]
            self.input_received.clear()

    def set_wheels(self, wheel_speeds):
        for i, wheel_speed in enumerate(wheel_speeds):
            wheel_speed = min(self.max_wheel_speed, max(-self.max_wheel_speed, wheel_speed))
            self.power[i] = wheel_speed / self.max_wheel_speed

        if self.check_input:
            self.last_update = rospy.get_rostime()
            self.input_received.set()
