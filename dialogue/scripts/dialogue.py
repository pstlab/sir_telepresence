#!/usr/bin/env python
import rospy
import sys
import speech_recognition as sr
import gtts
import requests
import json
from playsound import playsound
from msgs.srv import start_dialogue, start_dialogueResponse
from std_srvs.srv import Trigger, TriggerResponse

stt = sr.Recognizer()
stt_active = False
dialogue_id = -1


def start_listening(req):
    stt_active = True
    return TriggerResponse(success=True, message='Started listening')


def listen():
    with sr.Microphone() as source:
        stt.adjust_for_ambient_noise(source)
        rospy.logdebug('listening..')
        audio = stt.listen(source)
        rospy.logdebug('processing..')
        try:
            recognized_text = stt.recognize_google(audio, language='it-IT')
            stt_active = False
            rospy.logdebug('Decoded speech: %s', recognized_text)
        except Exception as e:
            print(e)
    return


def say(text):
    tts = gtts.gTTS(text, lang='it')
    tts.save('utterance.mp3')
    playsound('utterance.mp3')
    return


def start_new_dialogue(req):
    dialogue_id = req.dialogue_id
    return start_dialogueResponse(0)


def main(args):
    rospy.init_node('dialogue', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    sd = rospy.Service('start_dialogue', start_dialogue, start_new_dialogue)
    sl = rospy.Service('listen', Trigger, start_listening)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')


if __name__ == '__main__':
    main(sys.argv)
