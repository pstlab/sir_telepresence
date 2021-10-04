#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse


def activate_microphone(req):
    return TriggerResponse(success=True, message='Microphone activated..')


if __name__ == '__main__':
    rospy.init_node('speech_to_text', anonymous=True)
    rospy.loginfo('Starting Speech to Text Manager..')

    service = rospy.Service('activate_microphone',
                            Trigger, activate_microphone)

    rospy.spin()
