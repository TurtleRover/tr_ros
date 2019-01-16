import rospy
from robot import Robot


def main():

    rospy.init_node('tr_control')

    rate = rospy.Rate(10)
    r = Robot()

    while not rospy.is_shutdown():
        r.send_payload()
        rate.sleep()


if __name__ == '__main__':
    main()
