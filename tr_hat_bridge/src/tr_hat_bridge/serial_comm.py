import rospy

import serial


class SerialComm():
    def __init__(self, device, baudrate=115200, timeout=1.0):
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def send(self, data):
        self.serial.write(data)

    def readline(self):
        try:
            return self.serial.readline()
        except serial.SerialException as e:
            rospy.logerr(e)

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
