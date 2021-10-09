#!/usr/bin/env python3
import rospy
import gtts
import playsound
from msgs.srv import string_service, string_serviceResponse


utterance = ''


def speak(srv):
    utterance = srv.text
    rospy.logdebug('synthesizing "%s"..', utterance)
    return string_serviceResponse(True)


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True)
    rospy.loginfo('Starting Text to Speech Manager..')

    speak_service = rospy.Service('activate_speaker', string_service, speak)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if(utterance):
            tts = gtts.gTTS(utterance, lang="it")
            tts.save("utterance.mp3")
            playsound("utterance.mp3")
            utterance = ''
        rate.sleep()
