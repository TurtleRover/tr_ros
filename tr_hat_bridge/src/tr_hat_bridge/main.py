import rospy

from bridge import Bridge


def main():

    rospy.init_node('tr_hat_bridge')

    Bridge()

    rospy.spin()


if __name__ == '__main__':
    main()
