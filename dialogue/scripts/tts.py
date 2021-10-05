#!/usr/bin/env python3
import rospy
from msgs.srv import string_service, string_serviceResponse


speaking = False


def speak(srv):
    speaking = True
    rospy.logdebug('synthesizing "%s"..', srv.text)
    return string_serviceResponse(True)


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True)
    rospy.loginfo('Starting Text to Speech Manager..')

    speak_service = rospy.Service('activate_speaker', string_service, speak)

    rospy.spin()
