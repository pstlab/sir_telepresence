#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from msgs.srv import get_string, get_stringResponse
from std_srvs.srv import Empty, EmptyResponse


class speech_to_text:

    def __init__(self):
        self.recognizer_instance = sr.Recognizer()
        self.recognizer_instance.dynamic_energy_threshold = False
        self.adjusting = False

        configure_service = rospy.Service('configure_speech_to_text',
                                          Empty, self.configure_speech_to_text)
        mic_service = rospy.Service('speech_to_text',
                                    get_string, self.speech_to_text)

    def configure_speech_to_text(self, req):
        rospy.logdebug('adjusting for ambient noise..')
        with sr.Microphone() as source:
            self.recognizer_instance.adjust_for_ambient_noise(
                source, duration=2)
            rospy.logdebug('current energy threshold is ' +
                           str(self.recognizer_instance.energy_threshold))
        return EmptyResponse()

    def speech_to_text(self, req):
        rospy.logdebug('activating microphone..')
        with sr.Microphone() as source:
            audio = self.recognizer_instance.listen(source)
            try:
                rospy.logdebug('recognizing..')
                text = self.recognizer_instance.recognize_google(
                    audio, language="it-IT")
                rospy.logdebug('recognized speech: %s', text)
                return get_stringResponse(True, text)
            except Exception as e:
                print(e)
        return get_stringResponse(False, '')
