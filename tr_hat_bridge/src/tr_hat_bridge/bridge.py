import rospy
from std_msgs.msg import Float32, Int16, Float32MultiArray, String

import struct
from threading import Thread

import frame
from serial_comm import SerialComm
from utils import power_to_motor_payload, servo_angle_to_duty, reset_STM


class Bridge():
    def __init__(self):

        self.serial_device = rospy.get_param("~device", "/dev/ttyAMA0")
        self.battery_pub_rate = rospy.get_param("~battery_pub_rate", 1.0)

        servo1_min_angle = rospy.get_param("~servo1_min_angle", -90)
        servo1_max_angle = rospy.get_param("~servo1_max_angle", 90)
        servo1_min_duty = rospy.get_param("~servo1_min_duty", 2400)
        servo1_max_duty = rospy.get_param("~servo1_max_duty", 4800)

        servo2_min_angle = rospy.get_param("~servo2_min_angle", -90)
        servo2_max_angle = rospy.get_param("~servo2_max_angle", 90)
        servo2_min_duty = rospy.get_param("~servo2_min_duty", 2400)
        servo2_max_duty = rospy.get_param("~servo2_max_duty", 4800)

        servo3_min_angle = rospy.get_param("~servo3_min_angle", -90)
        servo3_max_angle = rospy.get_param("~servo3_max_angle", 90)
        servo3_min_duty = rospy.get_param("~servo3_min_duty", 2400)
        servo3_max_duty = rospy.get_param("~servo3_max_duty", 4800)

        self.comm = SerialComm(self.serial_device)
        reset_STM()
        self.comm.connect()

        self.motor_sub = rospy.Subscriber(
            "~motors",
            Float32MultiArray,
            self.set_motors
        )

        self.servo1_sub = rospy.Subscriber(
            "servo1/angle",
            Int16,
            self.get_servo_callback(
                1,
                servo1_min_angle,
                servo1_max_angle,
                servo1_min_duty,
                servo1_max_duty
            )
        )

        self.servo2_sub = rospy.Subscriber(
            "servo2/angle",
            Int16,
            self.get_servo_callback(
                2,
                servo2_min_angle,
                servo2_max_angle,
                servo2_min_duty,
                servo2_max_duty
            )
        )

        self.servo3_sub = rospy.Subscriber(
            "servo3/angle",
            Int16,
            self.get_servo_callback(
                3,
                servo3_min_angle,
                servo3_max_angle,
                servo3_min_duty,
                servo3_max_duty
            )
        )

        self.firmware_ver_pub = rospy.Publisher(
            "~firmware_version",
            String,
            latch=True,
            queue_size=1
        )

        self.battery_pub = rospy.Publisher(
            "battery",
            Float32,
            queue_size=1
        )

        self.publish_firmware_ver()

        self.publish_battery_thread = Thread(target=self.publish_battery_loop)
        self.publish_battery_thread.daemon = True
        self.publish_battery_thread.start()

    def set_motors(self, msg):
        if len(msg.data) < 4:
            rospy.logerr("Wrong array size in motor command")
            return

        payload = []
        for p in msg.data:
            value = power_to_motor_payload(p)
            payload.append(value)

        f = frame.motors(payload)
        status = self.comm.proccess_command(f)

        if not status or not status == " OK \r\n":
            rospy.logerr("Did not receive a valid response after a motor command")

    def get_servo_callback(self, channel, min_angle, max_angle, min_duty, max_duty):
        def set_servo(msg):
            angle = msg.data
            duty = servo_angle_to_duty(angle, min_angle, max_angle, min_duty, max_duty)

            f = frame.servo(channel, duty)
            status = self.comm.proccess_command(f)

            if not status or not status == " OK \r\n":
                rospy.logerr("Did not receive a valid response after servo command")

        return set_servo

    def publish_firmware_ver(self):
        firmware_ver = self.comm.proccess_command(frame.firmware_ver())

        if not firmware_ver or not firmware_ver.endswith("\r\n"):
            rospy.logerr("Could not get firmware version")
        else:
            firmware_ver = firmware_ver[:-2]
            self.firmware_ver_pub.publish(firmware_ver)

    def publish_battery(self):
        status = self.comm.proccess_command(frame.battery())

        if not status or not status.endswith("\r\n"):
            rospy.logerr("Could not get battery status")
        else:
            battery_status = struct.unpack("<f", status[:4])[0]
            self.battery_pub.publish(battery_status)

    def publish_battery_loop(self):
        rate = rospy.Rate(self.battery_pub_rate)
        while not rospy.is_shutdown():
            self.publish_battery()
            rate.sleep()
