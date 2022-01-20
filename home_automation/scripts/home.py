#!/usr/bin/env python3
import rospy
from home_automation.msg import home_automation_state


class home:

    def __init__(self):
        # publishes the state of the home automation manager..
        self.state_pub = rospy.Publisher(
            'home_automation_state', home_automation_state, queue_size=10, latch=True)
        self.state_pub.publish(
            home_automation_state(home_automation_state.idle))

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('home_automation',
                    anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Home Automation Manager..')

    hm = home()
    hm.start()
