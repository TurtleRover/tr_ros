import rospy
from geometry_msgs.msg import Twist, TwistStamped

from tr_hat_msgs.msg import MotorPayload

from driver import DriverStraight, DriverDifferential
from controller import Controller


class Robot():
    def __init__(self):
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.06)
        self.wheel_track = rospy.get_param("~wheel_track", 0.33)
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 6.0)
        self.differential_drive = rospy.get_param("~differential_drive", True)
        self.input_timeout = rospy.get_param("~input_timeout", 0.5)
        self.timestamp_check = rospy.get_param("~timestamp_check", 0.0)

        if self.differential_drive:
            Driver = DriverDifferential
        else:
            Driver = DriverStraight

        self.driver = Driver(
            wheel_radius=self.wheel_radius,
            wheel_track=self.wheel_track
        )

        self.controller = Controller(
            max_wheel_speed=self.max_wheel_speed,
            input_timeout=self.input_timeout
        )

        self.cmd_sub = rospy.Subscriber(
            "cmd_vel",
            Twist,
            self.callback_cmd,
            queue_size=1
        )

        self.cmd_stamped_sub = rospy.Subscriber(
            "cmd_vel_stamped",
            TwistStamped,
            self.callback_cmd_stamped,
            queue_size=1
        )

        self.motor_pub = rospy.Publisher(
            "tr_hat_bridge/motors",
            MotorPayload,
            queue_size=1
        )

    def callback_cmd(self, twist):
        linear = twist.linear.x
        angular = twist.angular.z

        self.driver.set_motors(linear, angular)
        self.controller.set_wheels(self.driver.wheel_speeds)

    def callback_cmd_stamped(self, data):
        if self.timestamp_check > 0:
            diff_sec = (rospy.get_rostime() - data.header.stamp).to_sec()
            if diff_sec < 0.0:
                rospy.logerr("Received command from the future! Make sure "
                             "the time is synchronized or disable checking")
                return
            if diff_sec > self.timestamp_check:
                rospy.logwarn(("Received a velocity command that is {0} seconds "
                               "in the past. Ignoring").format(diff_sec))
                return

        self.callback_cmd(data.twist)

    def send_payload(self):
        self.motor_pub.publish(self.controller.payload)
