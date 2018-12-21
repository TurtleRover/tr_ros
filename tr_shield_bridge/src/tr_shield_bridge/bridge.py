import rospy

import frame
from uart import Uart

from tr_msgs.msg import MotorPayload

class Bridge():
    def __init__(self):
        self.uart = Uart()
        self.uart.connect()

        self.motor_sub = rospy.Subscriber(
            "~motors", 
            MotorPayload, 
            self.callback_motors
        )

    def callback_motors(self, data):
        f = frame.motors(data.payload)
        self.uart.send(f)