import rospy
from driver import Driver

def main():

    rospy.init_node('tr_control')

    rate = rospy.Rate(10)
    d = Driver()    

    while not rospy.is_shutdown():
        d.send_payload()
        rate.sleep()

if __name__ == '__main__':
    main()