#!/usr/bin/env python3
import rospy
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse, string_service, string_serviceResponse
from std_srvs.srv import Trigger, TriggerResponse


utterance = ''


def speak(text):
    if speaker_proxy.call(string_service(text)):
        rospy.logdebug('speaking..')
        state.publish(dialogue_state(dialogue_state.speaking))


def listen(req):
    if microphone_proxy.call():
        rospy.logdebug('listening..')
        state.publish(dialogue_state(dialogue_state.listening))
        return TriggerResponse(True)
    else:
        return TriggerResponse(False)


def start_dialogue(req):
    return start_taskResponse(True)


def generate_response(req):
    utterance = req.text
    rospy.logdebug('generating response for "%s"..', utterance)
    return string_serviceResponse(True)


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    # publishes the state of the dialogue manager..
    state = rospy.Publisher('dialogue_state', dialogue_state, queue_size=10)
    # called by the deliberative tier..
    dialogue_service = rospy.Service(
        'start_dialogue', start_task, start_dialogue)
    # called by the gui for starting a dialogue by the user..
    listen_service = rospy.Service('listen', Trigger, listen)
    # called by the speech to text..
    response_service = rospy.Service(
        'produce_response', string_service, generate_response)

    # activates the speech to text..
    microphone_proxy = rospy.ServiceProxy('activate_microphone', Trigger)
    # activates the text to speech..
    speaker_proxy = rospy.ServiceProxy('activate_speaker', Trigger)

    rospy.spin()
