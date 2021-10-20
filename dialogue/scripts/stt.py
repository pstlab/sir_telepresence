#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from msgs.srv import string_service
from std_srvs.srv import Trigger, TriggerResponse


class speech_to_text:

    def __init__(self):
        self.recognizer_instance = sr.Recognizer()
        self.recognizer_instance.dynamic_energy_threshold = False
        self.stt_active = False
        self.adjusting = False

        mic_service = rospy.Service('open_microphone',
                                    Trigger, self.open_microphone_request)
        adj_service = rospy.Service('adjust_microphone',
                                    Trigger, self.adjust_microphone_request)

        self.generate_responses = rospy.ServiceProxy(
            'generate_responses', string_service)

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if(self.adjusting):
                self.adjust_energy_threshold()
                self.adjusting = False
            elif(self.stt_active):
                self.listen()
                self.stt_active = False
            rate.sleep()

    def listen(self):
        rospy.logdebug('activating microphone..')
        with sr.Microphone() as source:
            audio = self.recognizer_instance.listen(source)
            try:
                rospy.logdebug('recognizing..')
                text = self.recognizer_instance.recognize_google(
                    audio, language="it-IT")
                rospy.logdebug('recognized speech: %s', text)
                self.generate_responses(text)
            except Exception as e:
                print(e)

    def adjust_energy_threshold(self):
        rospy.logdebug('adjusting for ambient noise..')
        with sr.Microphone() as source:
            self.recognizer_instance.adjust_for_ambient_noise(
                source, duration=2)
            rospy.logdebug('current energy threshold is ' +
                           str(self.recognizer_instance.energy_threshold))

    def open_microphone_request(self, req):
        self.stt_active = True
        rospy.logdebug('request for activating microphone..')
        return TriggerResponse(True, 'Microphone activation request successful')

    def adjust_microphone_request(self, req):
        self.adjusting = True
        rospy.logdebug('request for adjusting microphone..')
        return TriggerResponse(True, 'Microphone adjustment request successful')


if __name__ == '__main__':
    rospy.init_node('speech_to_text', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Speech to Text Manager..')

    stt = speech_to_text()
    stt.start()
