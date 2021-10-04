#!/usr/bin/env python3
import rospy


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True)
    rospy.loginfo('Starting Text to Speech Manager..')

    rospy.spin()
