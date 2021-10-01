#!/usr/bin/env python3
import rospy
import sys
import gtts
from playsound import playsound
from speak.srv import Speak, SpeakResponse


def speak(req):
    tts = gtts.gTTS(req.text, lang="it")
    tts.save("utterance.mp3")
    playsound("utterance.mp3")
    return SpeakResponse(0)


def main(args):
    rospy.init_node('dialogue', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    s = rospy.Service('add_two_ints', Speak, speak)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')


if __name__ == '__main__':
    main(sys.argv)
