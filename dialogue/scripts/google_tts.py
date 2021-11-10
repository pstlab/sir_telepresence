#!/usr/bin/env python3
import rospy
import gtts
from playsound import playsound
from msgs.srv import set_string, set_stringResponse
import os #adding alternatively to playsound 

class text_to_speech:

    def __init__(self):
        text_to_speech_service = rospy.Service(
            'text_to_speech', set_string, self.tts)

    def tts(self, req):
        rospy.logdebug('synthesizing "%s"..', req.text)
        tts = gtts.gTTS(req.text, lang='it')
        tts.save('utterance.mp3')
        #playsound('utterance.mp3')
        os.system("mpg123 " + "utterance.mp3")
        return set_stringResponse(True)


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Text to Speech Manager..')

    tts = text_to_speech()
    rospy.spin()
~                