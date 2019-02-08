from __future__ import with_statement

import rospy
from tr_hat_msgs.msg import MotorPower, ServoAngle
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

        self.lock = Lock()

        self.motor_sub = rospy.Subscriber(
            "~motors",
            MotorPower,
            self.set_motors
        )

        self.servo_sub = rospy.Subscriber(
            "~servo",
            ServoAngle,
            self.set_servo
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
        payload = []
        for p in data.power:
            p = max(min(p, 1.0), -1.0)
            value = int(round(p * 0x7F))
            if value < 0:
                value = -value + 0x7F
            payload.append(value)

        with self.lock:
            self.comm.serial.flushInput()
            f = frame.motors(payload)
            self.comm.send(f)
            status = self.comm.readline()

        if not status or not status == " OK \r\n":
            rospy.logerr("Did not receive a valid response after motor command")

    def set_servo(self, data):
        if data.channel not in [1,2,3]:
            rospy.logerr(("Wrong servo channel! Received {0}, "
                          "expected 1, 2 or 3".format(data.channel)))
            return

        value = int((data.angle / 180.0) * 3450 + 1300)
        duty = struct.pack(">H", value)
        f = frame.servo(data.channel, duty)

        with self.lock:
            self.comm.serial.flushInput()
            self.comm.send(f)
            status = self.comm.readline()

        if not status or not status == " OK \r\n":
            rospy.logerr("Did not receive a valid response after servo command")

    def get_battery(self, data):
        with self.lock:
            self.comm.serial.flushInput()
            self.comm.send(frame.battery())
            #status = self.comm.readline()
            status = self.comm.serial.read(4)

        if not status:  #or not status.endswith("\r\n"):
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
