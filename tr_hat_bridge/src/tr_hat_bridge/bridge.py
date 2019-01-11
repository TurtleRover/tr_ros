from __future__ import with_statement

import rospy
from tr_hat_msgs.msg import MotorPayload
from tr_hat_msgs.srv import (GetBattery, GetBatteryResponse,
                             GetFirmwareVer, GetFirmwareVerResponse)

import struct
from threading import Lock

import frame
from serial_comm import SerialComm


class Bridge():
    def __init__(self):

        serial_device = rospy.get_param("~device", "/dev/ttyAMA0")

        self.comm = SerialComm(serial_device)
        self.comm.connect()

        self.srv_lock = Lock()

        self.motor_sub = rospy.Subscriber(
            "~motors",
            MotorPayload,
            self.set_motors
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

    def set_motors(self, data):
        f = frame.motors(data.payload)
        self.comm.send(f)

    def get_battery(self, data):
        with self.srv_lock:
            self.comm.send(frame.battery())
            status = self.comm.serial.read(1)

        if not status:
            success = False
            rospy.logerr("Could not get battery status")
            battery_status = 0
        else:
            success = True
            battery_status = struct.unpack(">B", status)[0]

        return GetBatteryResponse(success, battery_status)

    def get_firmware_ver(self, data):
        with self.srv_lock:
            self.comm.send(frame.firmware_ver())
            firmware_ver = self.comm.readline()

        if not firmware_ver:
            success = False
            rospy.logerr("Could not get firmware version")
        else:
            success = True

        return GetFirmwareVerResponse(success, firmware_ver)
