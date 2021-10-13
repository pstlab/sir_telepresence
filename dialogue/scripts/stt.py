#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from msgs.srv import string_service
from std_srvs.srv import Trigger, TriggerResponse


recognizer_instance = sr.Recognizer()
stt_active = False


def activate_microphone(req):
    global stt_active
    stt_active = True
    rospy.logdebug('activating microphone..')
    return TriggerResponse(True, 'Microphone activated')


if __name__ == '__main__':
    rospy.init_node('speech_to_text', anonymous=True)
    rospy.loginfo('Starting Speech to Text Manager..')

    service = rospy.Service('activate_microphone',
                            Trigger, activate_microphone)

    generate_responses = rospy.ServiceProxy(
        'generate_responses', string_service)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if(stt_active):
            with sr.Microphone() as source:
                rospy.logdebug('listening..')
                recognizer_instance.adjust_for_ambient_noise(source)
                audio = recognizer_instance.listen(source)
                stt_active = False
                try:
                    text = recognizer_instance.recognize_google(
                        audio, language="it-IT")
                    rospy.logdebug('decoded speech: %s', text)
                    generate_responses(text)
                except Exception as e:
                    print(e)
        rate.sleep()
