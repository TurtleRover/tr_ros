import rospy
from tr_hat_msgs.msg import MotorPayload
from tr_hat_msgs.srv import GetBattery, GetBatteryResponse

import struct

import frame
from uart import Uart

class Bridge():
    def __init__(self):
        self.uart = Uart()
        self.uart.connect()

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
        self.uart.send(f)

    def getBattery(self, data):
        self.uart.send(frame.battery())
        status = self.uart.serial.read(1)
        battery_status = struct.unpack(">B", status)[0]
        return GetBatteryResponse(battery_status)