#!/usr/bin/env python3
import rospy
import requests
import time
import traceback
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse, task_finished, state, get_string, set_string
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse

nobkg_idle = 'nobkg_idle'
nobkg_listening = 'nobkg_ascolto'


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
        listen_service = rospy.Service('listen', Trigger, self.listen)

        # retrieves the current emotions..
        self.perceive_emotions = rospy.ServiceProxy(
            'perceive_emotions', state)

        # activates the text to speech..
        self.text_to_speech = rospy.ServiceProxy(
            'text_to_speech', set_string)
        self.text_to_speech.wait_for_service()

        # activates the configuration of the speech to text..
        self.configure_speech_to_text = rospy.ServiceProxy(
            'configure_speech_to_text', Empty)
        self.configure_speech_to_text.wait_for_service()

        # activates the speech to text..
        self.speech_to_text = rospy.ServiceProxy(
            'speech_to_text', get_string)
        self.speech_to_text.wait_for_service()

        # activates the speech to text..
        self.set_face = rospy.ServiceProxy(
            'set_face', set_string)
        self.set_face.wait_for_service()

        # publishes the state of the dialogue manager..
        self.state_pub = rospy.Publisher(
            'dialogue_state', dialogue_state, queue_size=10, latch=True)
        self.state_pub.publish(dialogue_state(dialogue_state.idle))
        self.set_face(nobkg_idle)

    def start_dialogue(self, req):
        rospy.logdebug('Start task "%s" request..', req.task_name)
        # we store the informations about the starting dialogue task..
        self.task = True
        self.reasoner_id = req.reasoner_id
        self.task_id = req.task_id
        self.task_name = req.task_name
        return start_taskResponse(True)

    def listen(self, req):
        if self.user_dialogue:
            return TriggerResponse(False, 'Already listening..')
        elif self.task:
            return TriggerResponse(False, 'Already having a dialogue..')
        else:
            self.user_dialogue = True
            return TriggerResponse(True, 'Opening microphone..')

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.task_name:
                # we update the state..
                self.state_pub.publish(
                    dialogue_state(dialogue_state.configuring))
                self.set_face(nobkg_listening)
                # we configure the speech to text..
                try:
                    stt_conf = self.configure_speech_to_text()
                except rospy.ServiceException:
                    rospy.logerr('Speech to text configuration service call failed\n' +
                                 ''.join(traceback.format_stack()))

                # we add the current perceived emotions..
                try:
                    perceived_emotions = self.perceive_emotions()
                    self.par_names.extend(perceived_emotions.par_names)
                    self.par_values.extend(perceived_emotions.par_values)
                except rospy.ServiceException:
                    rospy.logerr('Emotions detection service call failed\n' +
                                 ''.join(traceback.format_stack()))

                # we prepare the request..
                payload = {'name': self.task_name}
                if self.par_names:
                    entities = {}
                    for i in range(len(self.par_names)):
                        entities[self.par_names[i]] = self.par_values[i]
                    payload['entities'] = entities

                # we make the request..
                rospy.logdebug(
                    'Generating responses for task "%s"..', self.task_name)
                try:
                    r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                                      '/trigger_intent', params={'include_events': 'NONE'}, json=payload)
                except requests.exceptions.RequestException as e:
                    rospy.logerr('Rasa server call failed\n' +
                                 ''.join(traceback.format_stack()))
                    raise SystemExit(e)

                if(r.status_code == requests.codes.ok):
                    # we update the state..
                    self.state_pub.publish(
                        dialogue_state(dialogue_state.speaking))
                    j_res = r.json()
                    self.state = j_res['tracker']['slots']
                    try:
                        for ans in j_res['messages']:
                            # self.set_face(ans['custom']['face'])
                            # self.text_to_speech(ans['custom']['text'])
                            self.set_face(nobkg_idle)
                            self.text_to_speech(ans['text'])
                    except rospy.ServiceException:
                        rospy.logerr('Text to speech service call failed\n' +
                                     ''.join(traceback.format_stack()))
                        # we update the state..
                        self.state_pub.publish(
                            dialogue_state(dialogue_state.idle))
                        self.set_face(nobkg_idle)
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
        self.set_face(nobkg_listening)
        # we listen..
        while True:
            stt = self.speech_to_text()
            if stt.text == '':
                try:
                    stt_conf = self.configure_speech_to_text()
                except rospy.ServiceException:
                    rospy.logerr('Speech to text configuration service call failed\n' +
                                 ''.join(traceback.format_stack()))
            else:
                break

        # we set the current perceived emotions..
        try:
            perceived_emotions = self.perceive_emotions()
            for i in range(len(perceived_emotions.par_names)):
                r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': perceived_emotions.par_names[i], 'value': perceived_emotions.par_values[i], 'timestamp': time.time()})
            if(r.status_code == requests.codes.ok):
                j_res = r.json()
        except rospy.ServiceException:
            rospy.logerr('Emotions detection service call failed\n' +
                         ''.join(traceback.format_stack()))
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        # we make the request..
        rospy.logdebug('Generating responses for utterance "%s"..', stt.text)
        r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
            'include_events': 'NONE'}, json={'sender': user, 'message': stt.text})
        if(r.status_code == requests.codes.ok):
            # we update the state..
            self.state_pub.publish(dialogue_state(dialogue_state.speaking))
            j_res = r.json()
            try:
                for ans in j_res:
                    # self.set_face(ans['custom']['face'])
                    # self.text_to_speech(ans['custom']['text'])
                    self.set_face(nobkg_idle)
                    self.text_to_speech(ans['text'])
            except rospy.ServiceException:
                rospy.logerr('Text to speech service call failed\n' +
                             ''.join(traceback.format_stack()))
                # we update the state..
                self.state_pub.publish(
                    dialogue_state(dialogue_state.idle))
                self.set_face(nobkg_idle)

            try:
                r = requests.get('http://' + host + ':' + port + '/conversations/' + user +
                                 '/tracker', params={'include_events': 'NONE'})
            except requests.exceptions.RequestException as e:
                rospy.logerr('Rasa server call failed\n' +
                             ''.join(traceback.format_stack()))
                raise SystemExit(e)

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
            self.set_face(nobkg_idle)
            return True
        else:
            # we are still talking..
            return False


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Dialogue Manager..')

    user = rospy.get_param('user')
    host = rospy.get_param('~host', 'localhost')
    port = str(rospy.get_param('~port', 5005))

    dm = dialogue_manager()
    dm.start()
