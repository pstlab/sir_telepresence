#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from std_srvs.srv import Trigger, TriggerResponse


recognizer_instance = sr.Recognizer()
stt_active = False


def activate_microphone(req):
    stt_active = True
    rospy.logdebug('activating microphone..')
    return TriggerResponse(success=True, message='Microphone activated..')


if __name__ == '__main__':
    rospy.init_node('speech_to_text', anonymous=True)
    rospy.loginfo('Starting Speech to Text Manager..')

    service = rospy.Service('activate_microphone',
                            Trigger, activate_microphone)

    dialogue_proxy = rospy.ServiceProxy('activate_microphone', Trigger)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if(stt_active):
            with sr.Microphone() as source:
                recognizer_instance.adjust_for_ambient_noise(source)
                audio = recognizer_instance.listen(source)
                stt_active = False
                try:
                    text = recognizer_instance.recognize_google(
                        audio, language="it-IT")
                    print("Decoded speech: \n", text)
                    msg = String()
                    msg.data = text
                    self.pub.publish(msg)

                except Exception as e:
                    print(e)
