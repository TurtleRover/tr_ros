import rospy
from tr_hat_msgs.msg import MotorPayload
from tr_hat_msgs.srv import GetBattery, GetBatteryResponse

import struct

import frame
from serial_comm import SerialComm


class Bridge():
    def __init__(self):

        serial_device = rospy.get_param("~device", "/dev/ttyAMA0")
        
        self.comm = SerialComm(serial_device)
        self.comm.connect()

        self.motor_sub = rospy.Subscriber(
            "~motors",
            MotorPayload,
            self.setMotors
        )

        self.battery_srv = rospy.Service(
            "~get_battery",
            GetBattery,
            self.getBattery
        )

    def setMotors(self, data):
        f = frame.motors(data.payload)
        self.comm.send(f)

    def getBattery(self, data):
        self.comm.send(frame.battery())
        status = self.comm.serial.read(1)
        battery_status = struct.unpack(">B", status)[0]
        return GetBatteryResponse(battery_status)
