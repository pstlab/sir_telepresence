#!/usr/bin/env python3
import rospy
import requests
import json
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse, task_finished, string_service, string_serviceResponse, pronounce_responses, pronounce_responsesRequest
from std_srvs.srv import Trigger, TriggerResponse


class dialogue_manager:

    def __init__(self):
        self.slots = {}
        self.task = False
        self.reasoner_id = -1
        self.task_id = -1
        self.task_name = ''

        # called by the deliberative tier..
        start_dialogue_service = rospy.Service(
            'start_dialogue', start_task, self.start_dialogue_task)

        # notifies the deliberative tier that a dialogue task has finished..
        self.close_task = rospy.ServiceProxy('task_finished', task_finished)

        # called by the gui for starting a dialogue by the user..
        self.listen_service = rospy.Service('listen', Trigger, self.listen)

        # called by the speech to text for generating responses..
        generate_responses_service = rospy.Service(
            'generate_responses', string_service, self.generate_responses)

        # called by the text to speech..
        check_closed_dialogue_service = rospy.Service('check_closed_dialogue',
                                                      Trigger, self.check_closed_dialogue)

        # activates the speech to text..
        self.open_microphone = rospy.ServiceProxy('open_microphone', Trigger)

        # activates the speech to text..
        self.adjust_microphone = rospy.ServiceProxy(
            'adjust_microphone', Trigger)

        # activates the text to speech..
        self.pronounce_responses = rospy.ServiceProxy(
            'pronounce_responses', pronounce_responses)

        # publishes the state of the dialogue manager..
        self.state = rospy.Publisher(
            'dialogue_state', dialogue_state, queue_size=10, latch=True)
        self.state.publish(dialogue_state(dialogue_state.idle))

    def start():
        rospy.spin()

    def listen(self, req):
        return TriggerResponse(self.open_microphone())

    def activate_microphone(self):
        if self.open_microphone():
            rospy.logdebug('listening..')
            self.state.publish(dialogue_state(dialogue_state.listening))
            return True
        else:
            return False

    def start_dialogue_task(self, req):
        self.state.publish(dialogue_state(dialogue_state.speaking))

        # we adjust the speech to text for the ambient noise..
        self.adjust_microphone()

        # we store the informations about the starting dialogue task..
        self.task = True
        self.reasoner_id = req.reasoner_id
        self.task_id = req.task_id
        self.task_name = req.task_name

        # we prepare the request..
        payload = {'name': self.task_name}
        if req.par_names:
            entities = {}
            for i in range(len(req.par_names)):
                entities[req.par_names[i]] = req.par_values[i]
            payload['entities'] = entities

        r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                          '/trigger_intent', params={'include_events': 'NONE'}, json=self.payload)
        if(r.status_code == requests.codes.ok):
            j_res = r.json()
            self.slots = j_res['tracker']['slots']
            responses = pronounce_responsesRequest()
            for ans in j_res:
                responses.utterances.append(ans['text'])
            if self.pronounce_responses(responses):
                return start_taskResponse(True)
        return start_taskResponse(False)

    def generate_responses(self, req):
        self.state.publish(dialogue_state(dialogue_state.speaking))
        utterance = req.text
        rospy.logdebug('generating response for "%s"..', utterance)
        r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
            'include_events': 'NONE'}, json={'sender': user, 'message': utterance})
        if(r.status_code == requests.codes.ok):
            j_res = r.json()
            self.slots = j_res['slots']
            responses = pronounce_responsesRequest()
            for ans in j_res:
                responses.utterances.append(ans['text'])
            if self.pronounce_responses(responses):
                return start_taskResponse(True)
        return string_serviceResponse(False)

    def check_closed_dialogue(self, req):
        if self.slots['command_state'] == 'done' or self.slots['command_state'] == 'failure':
            if self.task:
                # we close the current task..
                if self.slots['command_state'] == 'done':
                    # the task is closed with a success..
                    self.close_task(
                        self.reasoner_id, self.task_id, True)
                elif self.slots['command_state'] == 'failure':
                    # the task is closed with a failure..
                    self.close_task(
                        self.reasoner_id, self.task_id, False)
                self.reasoner_id = -1
                self.task_id = -1
                self.task_name = ''
            self.state.publish(dialogue_state(dialogue_state.idle))
            return TriggerResponse(True)
        else:
            # we are still talking, so we reopen the microphone..
            return TriggerResponse(self.open_microphone())


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    host = rospy.get_param('rasa.host')
    port = rospy.get_param('rasa.port')
    user = rospy.get_param('user')

    dm = dialogue_manager()
    dm.start()
