#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from dialogue_manager.srv import get_string, get_stringResponse
from std_srvs.srv import Empty, EmptyResponse


class speech_to_text:

    def __init__(self):
        configure_service = rospy.Service('configure_speech_to_text',
                                          Empty, self.configure_stt)
        mic_service = rospy.Service('speech_to_text',
                                    get_string, self.stt)

    def configure_stt(self, req):
        rospy.logdebug('adjusting for ambient noise..')
        rec = sr.Recognizer()
        with sr.Microphone() as source:
            rec.adjust_for_ambient_noise(source, duration=1.0)
            if rec.energy_threshold >= 1000:
                rospy.logwarn('current energy threshold is ' +
                              str(rec.energy_threshold))
                rec.energy_threshold = 800
            self.energy_threshold = rec.energy_threshold
            rospy.logdebug('current energy threshold is ' +
                           str(self.energy_threshold))
        return EmptyResponse()

    def stt(self, req):
        rospy.logdebug('activating microphone..')
        rec = sr.Recognizer()
        rec.energy_threshold = self.energy_threshold
        with sr.Microphone() as source:
            audio = rec.listen(source, phrase_time_limit=10.0)
            try:
                rospy.logdebug('recognizing..')
                text = rec.recognize_google(
                    audio, language="it-IT")
                rospy.logdebug('recognized speech: %s', text)
                return get_stringResponse(True, text)
            except Exception as e:
                print(e)
        return get_stringResponse(False, '')


if __name__ == '__main__':
    rospy.init_node('speech_to_text', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Speech to Text Manager..')

    stt = speech_to_text()
    rospy.spin()
