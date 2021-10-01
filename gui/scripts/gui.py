#!/usr/bin/env python3
import rospy
import sys


def main(args):
    rospy.init_node('gui', anonymous=True)
    rospy.loginfo('Starting GUI..')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')


if __name__ == '__main__':
    main(sys.argv)
