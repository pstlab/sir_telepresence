#!/usr/bin/env python3
import rospy
import requests
import time
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse, task_finished, state, get_string, set_string
from std_srvs.srv import Empty, EmptyResponse


class dialogue_manager:

    def __init__(self):
        self.state = {}
        self.reasoner_id = -1
        self.task_id = -1
        self.task_name = ''
        self.par_names = []
        self.par_values = []
        self.user_dialogue = False

        # called by the deliberative tier..
        start_dialogue_service = rospy.Service(
            'start_dialogue', start_task, self.start_dialogue)

        # notifies the deliberative tier that a dialogue task has finished..
        self.close_task = rospy.ServiceProxy('task_finished', task_finished)

        # called by the gui for starting a dialogue by the user..
        listen_service = rospy.Service('listen', Empty, self.listen)

        # retrieves the current emotions..
        self.perceive_emotions = rospy.ServiceProxy(
            'perceive_emotions', state)

        # activates the text to speech..
        self.text_to_speech = rospy.ServiceProxy(
            'text_to_speech', set_string)

        # activates the speech to text..
        self.configure_speech_to_text = rospy.ServiceProxy(
            'configure_speech_to_text', Empty)

        # activates the speech to text..
        self.speech_to_text = rospy.ServiceProxy(
            'speech_to_text', get_string)

        # publishes the state of the dialogue manager..
        self.state_pub = rospy.Publisher(
            'dialogue_state', dialogue_state, queue_size=10, latch=True)
        self.state_pub.publish(dialogue_state(dialogue_state.idle))

    def start_dialogue(self, req):
        # we store the informations about the starting dialogue task..
        self.task = True
        self.reasoner_id = req.reasoner_id
        self.task_id = req.task_id
        self.task_name = req.task_name
        return start_taskResponse(True)

    def listen(self, req):
        self.user_dialogue = True
        return EmptyResponse()

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.task_name:
                # we update the state..
                self.state_pub.publish(
                    dialogue_state(dialogue_state.configuring))
                # we configure the speech to text..
                stt = self.configure_speech_to_text()

                # we add the current perceived emotions..
                try:
                    perceived_emotions = self.perceive_emotions()
                    self.par_names.extend(perceived_emotions.par_names)
                    self.par_values.extend(perceived_emotions.par_values)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

                # we prepare the request..
                payload = {'name': self.task_name}
                if self.par_names:
                    entities = {}
                    for i in range(len(self.par_names)):
                        entities[self.par_names[i]] = self.par_values[i]
                    payload['entities'] = entities

                # we make the request..
                rospy.logdebug(
                    'generating responses for "%s"..', self.task_name)
                r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                                  '/trigger_intent', params={'include_events': 'NONE'}, json=payload)

                if(r.status_code == requests.codes.ok):
                    j_res = r.json()
                    self.state = j_res['tracker']['slots']
                    for ans in j_res['messages']:
                        self.text_to_speech(ans['text'])
                    self.dialogue()

            elif self.user_dialogue:
                self.interact()
                self.dialogue()

            rate.sleep()

    def dialogue(self):
        while not self.close_dialogue():
            self.interact()

    def interact(self):
        # we update the state..
        self.state_pub.publish(dialogue_state(dialogue_state.listening))
        # we listen..
        stt = self.speech_to_text()

        # we set the current perceived emotions..
        try:
            perceived_emotions = self.perceive_emotions()
            for i in range(len(perceived_emotions.par_names)):
                r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': perceived_emotions.par_names[i], 'value': perceived_emotions.par_values[i], 'timestamp': time.time()})
            if(r.status_code == requests.codes.ok):
                j_res = r.json()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # we make the request..
        rospy.logdebug('generating responses for "%s"..', stt.text)
        r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
            'include_events': 'NONE'}, json={'sender': user, 'message': stt.text})
        if(r.status_code == requests.codes.ok):
            j_res = r.json()
            for ans in j_res:
                # we update the state..
                self.state_pub.publish(dialogue_state(dialogue_state.speaking))
                # we speak..
                self.text_to_speech(ans['text'])
            r = requests.get('http://' + host + ':' + port + '/conversations/' + user +
                             '/tracker', params={'include_events': 'NONE'})
            if(r.status_code == requests.codes.ok):
                j_res = r.json()
                self.state = j_res['slots']

    def close_dialogue(self):
        if self.state['command_state'] == 'done' or self.state['command_state'] == 'failure':
            if self.task:
                # we close the current task..
                if self.state['command_state'] == 'done':
                    # the task is closed with a success..
                    self.close_task(
                        self.reasoner_id, self.task_id, True)
                elif self.state['command_state'] == 'failure':
                    # the task is closed with a failure..
                    self.close_task(
                        self.reasoner_id, self.task_id, False)
                self.reasoner_id = -1
                self.task_id = -1
                self.task_name = ''
                self.par_names.clear()
                self.par_values.clear()
            elif self.user_dialogue:
                # we close the current dialogue..
                self.user_dialogue = False
            # we update the state..
            self.state_pub.publish(dialogue_state(dialogue_state.idle))
            return True
        else:
            # we are still talking..
            return False


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Dialogue Manager..')

    host = rospy.get_param('rasa.host')
    port = rospy.get_param('rasa.port')
    user = rospy.get_param('user')

    dm = dialogue_manager()
    dm.start()
