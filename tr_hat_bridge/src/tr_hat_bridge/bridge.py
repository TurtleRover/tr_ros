from __future__ import with_statement

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from tr_hat_msgs.srv import (GetBattery, GetBatteryResponse,
                             GetFirmwareVer, GetFirmwareVerResponse)

import struct
from threading import Lock

import frame
from serial_comm import SerialComm
from utils import power_to_motor_payload, servo_angle_to_duty


class Bridge():
    def __init__(self):

        serial_device = rospy.get_param("~device", "/dev/ttyAMA0")

        self.comm = SerialComm(serial_device)
        self.comm.connect()

        self.lock = Lock()

        self.motor_sub = rospy.Subscriber(
            "~motors",
            Float32MultiArray,
            self.set_motors
        )

        self.servo1_sub = rospy.Subscriber(
            "~servo1/angle",
            Float32,
            self.get_servo_callback(1)
        )

        self.servo2_sub = rospy.Subscriber(
            "~servo2/angle",
            Float32,
            self.get_servo_callback(2)
        )

        self.servo3_sub = rospy.Subscriber(
            "~servo3/angle",
            Float32,
            self.get_servo_callback(3)
        )

        self.battery_srv = rospy.Service(
            "~get_battery",
            GetBattery,
            self.get_battery
        )

        self.firmware_ver_srv = rospy.Service(
            "~get_firmware_ver",
            GetFirmwareVer,
            self.get_firmware_ver
        )

    def set_motors(self, msg):
        if len(msg.data) < 4:
            rospy.logerr("Wrong array size in motor command")
            return

        payload = []
        for p in msg.data:
            value = power_to_motor_payload(p)
            payload.append(value)

        f = frame.motors(payload)

        with self.lock:
            self.comm.serial.flushInput()
            self.comm.send(f)
            status = self.comm.readline()

        if not status or not status == " OK \r\n":
            rospy.logerr("Did not receive a valid response after a motor command")

    def get_servo_callback(self, channel):
        def set_servo(msg):
            angle = msg.data
            duty = servo_angle_to_duty(angle)

            f = frame.servo(channel, duty)

            with self.lock:
                self.comm.serial.flushInput()
                self.comm.send(f)
                status = self.comm.readline()

            if not status or not status == " OK \r\n":
                rospy.logerr("Did not receive a valid response after servo command")

        return set_servo

    def get_battery(self, data):
        with self.lock:
            self.comm.serial.flushInput()
            self.comm.send(frame.battery())
            # status = self.comm.readline()
            status = self.comm.serial.read(4)

        if not status:  # or not status.endswith("\r\n"):
            success = False
            rospy.logerr("Could not get battery status")
            battery_status = 0
        else:
            success = True
            battery_status = struct.unpack("<f", status[:4])[0]

        return GetBatteryResponse(success, battery_status)

    def get_firmware_ver(self, data):
        with self.lock:
            self.comm.serial.flushInput()
            self.comm.send(frame.firmware_ver())
            firmware_ver = self.comm.readline()

        if not firmware_ver or not firmware_ver.endswith("\r\n"):
            success = False
            rospy.logerr("Could not get firmware version")
        else:
            success = True
            firmware_ver = firmware_ver[:-2]

        return GetFirmwareVerResponse(success, firmware_ver)
