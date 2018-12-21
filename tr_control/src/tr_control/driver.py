import rospy
from geometry_msgs.msg import TwistStamped

from tr_msgs.msg import MotorPayload

class Driver():
    def __init__(self):
        self.seq = 0
        self.payload = [0x00,0x00,0x00,0x00] # FL, FR, RL, RR
        self.max_speed_linear = rospy.get_param("max_speed_linear", 0.7)
        self.max_speed_angular = rospy.get_param("max_speed_angular", 2.0)

        self.cmd_sub = rospy.Subscriber(
            "/cmd_vel",
            TwistStamped, 
            self.callback_cmd
        )

        self.motor_pub = rospy.Publisher(
            "/tr_shield_bridge/motors", 
            MotorPayload, 
            queue_size=1
        )

    def callback_cmd(self, data):
        if data.header.seq < self.seq:
            return

        self.seq = data.header.seq

        linear = data.twist.linear.x
        angular = data.twist.angular.z

        if abs(linear) >= abs(angular):

            speed_linear = min(abs(linear), self.max_speed_linear)
            wheel_payload = int((speed_linear / self.max_speed_linear) * 0x7F)

            if linear < 0 and wheel_payload > 0:
                wheel_payload += 0x7F

            self.payload = [wheel_payload] * 4

        else:

            speed_angular = min(abs(angular), self.max_speed_angular)
            wheel_payload = int((speed_angular / self.max_speed_angular) * 0x7F)

            if wheel_payload == 0:
                wheel_left_payload = wheel_right_payload = 0
            elif angular > 0:
                wheel_left_payload = wheel_payload + 0x7F
                wheel_right_payload = wheel_payload
            else:
                wheel_left_payload = wheel_payload
                wheel_right_payload = wheel_payload + 0x7F

            self.payload = [wheel_left_payload, wheel_right_payload] * 2

    def send_payload(self):
        self.motor_pub.publish(self.payload)