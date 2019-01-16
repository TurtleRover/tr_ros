import rospy
from geometry_msgs.msg import TwistStamped

from tr_hat_msgs.msg import MotorPayload

from driver import DriverStraight, DriverDifferential
from controller import Controller


class Robot():
    def __init__(self):
        self.seq = 0
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.06)
        self.wheel_track = rospy.get_param("~wheel_track", 0.33)
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 6.0)
        self.differential_drive = rospy.get_param("~differential_drive", True)

        if self.differential_drive:
            Driver = DriverDifferential
        else:
            Driver = DriverStraight

        self.driver = Driver(
            wheel_radius=self.wheel_radius,
            wheel_track=self.wheel_track
        )

        self.controller = Controller(
            max_wheel_speed=self.max_wheel_speed
        )

        self.cmd_sub = rospy.Subscriber(
            "cmd_vel",
            TwistStamped,
            self.callback_cmd,
            queue_size=1
        )

        self.motor_pub = rospy.Publisher(
            "tr_hat_bridge/motors",
            MotorPayload,
            queue_size=1
        )

    def callback_cmd(self, data):
        if data.header.seq < self.seq:
            return

        self.seq = data.header.seq

        linear = data.twist.linear.x
        angular = data.twist.angular.z

        self.driver.set_motors(linear, angular)
        self.controller.set_wheels(self.driver.wheel_speeds)

    def send_payload(self):
        self.motor_pub.publish(self.controller.payload)
