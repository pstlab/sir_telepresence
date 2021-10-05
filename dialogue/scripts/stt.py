#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from std_srvs.srv import Trigger, TriggerResponse


recognizer_instance = sr.Recognizer()
stt_active = False


def activate_microphone(req):
    stt_active = True
    return TriggerResponse(success=True, message='Microphone activated..')


if __name__ == '__main__':
    rospy.init_node('speech_to_text', anonymous=True)
    rospy.loginfo('Starting Speech to Text Manager..')

    service = rospy.Service('activate_microphone',
                            Trigger, activate_microphone)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if(stt_active):
            with sr.Microphone() as source:
                recognizer_instance.adjust_for_ambient_noise(source)
