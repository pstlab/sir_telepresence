#!/usr/bin/env python3
import rospy
import requests
import json
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse, task_finished, string_service, string_serviceResponse, reproduce_responses, reproduce_responsesRequest
from std_srvs.srv import Trigger, TriggerResponse


slots = {}
reasoner_id = -1
task_id = -1
task_name = ''


def listen(req):
    if activate_microphone():
        rospy.logdebug('listening..')
        state.publish(dialogue_state(dialogue_state.listening))
        return TriggerResponse(True)
    else:
        return TriggerResponse(False)


def start_dialogue(req):
    state.publish(dialogue_state(dialogue_state.speaking))
    global slots, reasoner_id, task_id, task_name
    reasoner_id = req.request.reasoner_id
    task_id = req.request.task_id
    task_name = req.request.task_name
    r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                      '/trigger_intent', params={'include_events': 'NONE'}, json=task_name)
    if(r.status_code == requests.codes.ok):
        j_res = r.json()
        slots = j_res['tracker']['slots']
        responses = reproduce_responsesRequest()
        for ans in j_res:
            responses.utterances.append(ans['text'])
        return start_taskResponse(responses_proxy(responses))
    return start_taskResponse(False)


def generate_responses(req):
    state.publish(dialogue_state(dialogue_state.speaking))
    global slots
    utterance = req.text
    rospy.logdebug('generating response for "%s"..', utterance)
    r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
        'include_events': 'NONE'}, json={'sender': user, 'message': utterance})
    if(r.status_code == requests.codes.ok):
        j_res = r.json()
        slots = j_res['slots']
        responses = reproduce_responsesRequest()
        for ans in j_res:
            responses.utterances.append(ans['text'])
        return string_serviceResponse(responses_proxy(responses))
    return string_serviceResponse(True)


def generated_responses():
    if slots['command_state'] == 'done' or slots['command_state'] == 'failure':
        global reasoner_id, task_id, task_name
        if slots['command_state'] == 'done':
            task_finished_proxy(reasoner_id, task_id, True)
        elif slots['command_state'] == 'failure':
            task_finished_proxy(reasoner_id, task_id, False)
        reasoner_id = -1
        task_id = -1
        task_name = ''
        state.publish(dialogue_state(dialogue_state.idle))
    else:
        # we reactivate the microphone..
        listen()


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    # publishes the state of the dialogue manager..
    state = rospy.Publisher(
        'dialogue_state', dialogue_state, queue_size=10, latch=True)
    state.publish(dialogue_state(dialogue_state.idle))

    # called by the deliberative tier..
    dialogue_service = rospy.Service(
        'start_dialogue', start_task, start_dialogue)

    task_finished_proxy = rospy.ServiceProxy('task_finished', task_finished)

    # called by the gui for starting a dialogue by the user..
    listen_service = rospy.Service('listen', Trigger, listen)

    # called by the speech to text..
    generate_responses_service = rospy.Service(
        'generate_responses', string_service, generate_responses)

    # called by the text to speech..
    generated_responses_service = rospy.Service('generated_responses',
                                                Trigger, generated_responses)

    # activates the speech to text..
    activate_microphone = rospy.ServiceProxy('activate_microphone', Trigger)

    # activates the text to speech..
    responses_proxy = rospy.ServiceProxy(
        'reproduce_responses', reproduce_responses)

    host = rospy.get_param('rasa.host')
    port = rospy.get_param('rasa.port')
    user = rospy.get_param('user')

    rospy.spin()
