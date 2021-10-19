#!/usr/bin/env python3
import rospy
import requests
import time
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse, task_finished, string_service, string_serviceResponse, reproduce_responses, reproduce_responsesRequest, state
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
        listen_service = rospy.Service('listen', Trigger, self.listen)

        # called by the speech to text for generating responses..
        generate_responses_service = rospy.Service(
            'generate_responses', string_service, self.generate_responses)

        # called by the text to speech..
        check_closed_dialogue_service = rospy.Service('check_closed_dialogue',
                                                      Trigger, self.check_closed_dialogue)

        # adjusts the microphone of the speech to text..
        self.adjust_microphone = rospy.ServiceProxy(
            'adjust_microphone', Trigger)

        # activates the speech to text..
        self.open_microphone = rospy.ServiceProxy('open_microphone', Trigger)

        # activates the text to speech..
        self.reproduce_responses = rospy.ServiceProxy(
            'reproduce_responses', reproduce_responses)

        # retrieves the current emotions..
        self.emotions = rospy.ServiceProxy(
            'emotions', state)

        # publishes the state of the dialogue manager..
        self.state_pub = rospy.Publisher(
            'dialogue_state', dialogue_state, queue_size=10, latch=True)
        self.state_pub.publish(dialogue_state(dialogue_state.idle))

    def start(self):
        rospy.spin()

    def listen(self, req):
        return TriggerResponse(self.open_microphone(), 'Opening microphone')

    def activate_microphone(self):
        try:
            open_mic = self.open_microphone()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        if open_mic:
            rospy.logdebug('listening..')
            self.state_pub.publish(dialogue_state(dialogue_state.listening))
            return True
        else:
            return False

    def start_dialogue_task(self, req):
        self.state_pub.publish(dialogue_state(dialogue_state.speaking))

        # we adjust the speech to text for the ambient noise..
        try:
            res = self.adjust_microphone()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # we store the informations about the starting dialogue task..
        self.task = True
        self.reasoner_id = req.reasoner_id
        self.task_id = req.task_id
        self.task_name = req.task_name

        # we add the current perceived emotions..
        try:
            emotions = self.emotions()
            req.par_names.extend(emotions.par_names)
            req.par_values.extend(emotions.par_values)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # we prepare the request..
        payload = {'name': self.task_name}
        if req.par_names:
            entities = {}
            for i in range(len(req.par_names)):
                entities[req.par_names[i]] = req.par_values[i]
            payload['entities'] = entities

        # we make the request..
        rospy.logdebug('generating responses for "%s"..', self.task_name)
        r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                          '/trigger_intent', params={'include_events': 'NONE'}, json=payload)
        if(r.status_code == requests.codes.ok):
            j_res = r.json()
            self.slots = j_res['tracker']['slots']
            responses = reproduce_responsesRequest()
            for ans in j_res['messages']:
                responses.utterances.append(ans['text'])
            try:
                res = self.reproduce_responses(responses)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            return start_taskResponse(res.started)
        return start_taskResponse(False)

    def generate_responses(self, req):
        self.state_pub.publish(dialogue_state(dialogue_state.speaking))

        # we set the current perceived emotions..
        try:
            emotions = self.emotions()
            for i in range(len(emotions.par_names)):
                slot_set = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': emotions.par_names[i], 'value': emotions.par_values[i], 'timestamp': time.time()}).json()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # we make the request..
        rospy.logdebug('generating responses for "%s"..', req.text)
        r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
            'include_events': 'NONE'}, json={'sender': user, 'message': req.text})
        if(r.status_code == requests.codes.ok):
            j_res = r.json()
            self.slots = j_res['slots']
            responses = reproduce_responsesRequest()
            for ans in j_res:
                responses.utterances.append(ans['text'])
            try:
                res = self.reproduce_responses(responses)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            return start_taskResponse(res)
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
            self.state_pub.publish(dialogue_state(dialogue_state.idle))
            return TriggerResponse(True, 'Closed dialogue')
        else:
            # we are still talking, so we reopen the microphone..
            return TriggerResponse(self.open_microphone(), 'Reopening microphone')


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    host = rospy.get_param('rasa.host')
    port = rospy.get_param('rasa.port')
    user = rospy.get_param('user')

    dm = dialogue_manager()
    dm.start()
