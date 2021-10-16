#!/usr/bin/env python3
from build.dialogue.catkin_generated.installspace.tts import speak
import rospy
import gtts
import playsound
from msgs.srv import reproduce_responses, reproduce_responsesResponse
from std_srvs.srv import Trigger


class text_to_speech:

    def __init__(self):
        self.utterances = []

        rep_res_service = rospy.Service(
            'reproduce_responses', reproduce_responses, self.speak_request)
        self.check_closed_dialogue = rospy.ServiceProxy(
            'check_closed_dialogue', Trigger)

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.utterances:
                speak()
                self.utterances.clear()
            rate.sleep()

    def speak(self):
        rospy.logdebug('synthesizing "%s"..', self.utterances)
        for utterance in self.utterances:
            tts = gtts.gTTS(utterance, lang="it")
            tts.save("utterance.mp3")
            playsound("utterance.mp3")
            try:
                res = self.check_closed_dialogue()
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

    def speak_request(self, srv):
        self.utterances = srv.utterances
        rospy.logdebug('request for synthesizing "%s"..', self.utterances)
        return reproduce_responsesResponse(True)


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True)
    rospy.loginfo('Starting Text to Speech Manager..')

    tts = text_to_speech()
    tts.start()
