#!/usr/bin/env python3
import rospy
import gtts
import playsound
from msgs.srv import reproduce_responses, reproduce_responsesResponse
from std_srvs.srv import Trigger


utterances = []


def speak(srv):
    global utterances
    utterances = srv.utterances
    rospy.logdebug('synthesizing "%s"..', utterances)
    return reproduce_responsesResponse(True)


if __name__ == '__main__':
    rospy.init_node('text_to_speech', anonymous=True)
    rospy.loginfo('Starting Text to Speech Manager..')

    reproduce_responses_service = rospy.Service(
        'reproduce_responses', reproduce_responses, speak)
    generated_responses = rospy.ServiceProxy('generated_responses', Trigger)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if utterances:
            for utterance in utterances:
                tts = gtts.gTTS(utterance, lang="it")
                tts.save("utterance.mp3")
                playsound("utterance.mp3")
                generated_responses()
            utterances.clear()
        rate.sleep()
