#!/usr/bin/env python3
import rospy
import gtts
#from playsound import playsound
import os  # adding alternatively to playsound
from dialogue_manager.srv import utterance_to_pronounce, utterance_to_pronounceResponse


class text_to_speech:

    def __init__(self):
        text_to_speech_service = rospy.Service(
            'text_to_speech', utterance_to_pronounce, self.tts)

    def tts(self, req):
        rospy.logdebug('synthesizing "%s"..', req.utterance)
        tts = gtts.gTTS(req.utterance, lang='it')
        tts.save('utterance.mp3')
        # playsound('utterance.mp3')
        os.system("mpg123 " + "utterance.mp3")
        return utterance_to_pronounceResponse(True)


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Text to Speech Manager..')

    tts = text_to_speech()
    rospy.spin()
