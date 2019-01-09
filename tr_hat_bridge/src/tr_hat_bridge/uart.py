import rospy

import serial
import serial.tools.list_ports


class Serial():
    def __init__(self):
        self.BAUD = 115200
        self.port = "AMA"
        self.serial = None

    def send(self, data):
        self.serial.write(data)

    def readline(self):
        try:
            return self.serial.readline()
        except serial.SerialException as e:
            rospy.logerror(e)
            return 0

    def connect(self):
        while self.serial is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for serial device")
            for port in self.available_ports():
                if self.port in port.name:
                    rospy.loginfo("Found device: %s %s",
                                  port.name, port.manufacturer)
                    try:
                        self.serial = serial.Serial(
                            port.device, baudrate=self.BAUD)
                        rospy.loginfo("Connected to device: %s", port.device)
                    except serial.SerialException as e:
                        rospy.logerror(e)

            rospy.sleep(1)

    def available_ports(self):
        return serial.tools.list_ports.comports()
