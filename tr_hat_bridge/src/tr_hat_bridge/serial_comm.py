from __future__ import with_statement

import rospy

import serial
from threading import Lock


class SerialComm():
    def __init__(self, device, baudrate=115200, timeout=1.0):
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.lock = Lock()

    def proccess_command(self, data):
        if not self.serial:
            rospy.logerr("Serial communication not yet initialized")
            return None

        with self.lock:
            self.serial.flushInput()
            self.serial.write(data)
            try:
                status = self.serial.readline()
            except serial.SerialException as e:
                rospy.logerr(e)
                return None
            else:
                return status

    def connect(self):
        while self.serial is None and not rospy.is_shutdown():
            try:
                self.serial = serial.Serial(
                    self.device,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                rospy.loginfo("Connected to serial device %s" % self.device)
            except serial.SerialException as e:
                rospy.logerr(e)
                rospy.loginfo("Waiting for serial device")
                rospy.sleep(1)
