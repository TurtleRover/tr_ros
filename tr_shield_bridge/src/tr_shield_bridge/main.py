import rospy

from bridge import Bridge

def main():

    rospy.init_node('tr_shield_bridge')

    bridge = Bridge()

    rospy.spin()

if __name__ == '__main__':
    main()