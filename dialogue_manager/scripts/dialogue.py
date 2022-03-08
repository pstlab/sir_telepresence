#!/usr/bin/env python3
import rospy
import requests
import time
import traceback
from std_srvs.srv import Trigger, TriggerResponse, Empty
from dialogue_manager.msg import dialogue_state, audio, video, button
from dialogue_manager.srv import utterance_to_pronounce, face_to_show, image_to_show, audio_to_play, video_to_play, page_to_show, question_to_ask, utterance_to_recognize
from deliberative_tier.msg import task
from deliberative_tier.srv import task_service, task_serviceResponse, task_finished
from persistence_manager.srv import get_state, set_state, set_stateResponse

face_idle = 'idle'
face_talking = 'talking'
face_listening = 'listening'
coherent = True


class dialogue_manager:

    def __init__(self):
        self.deliberative_task = False
        self.state = {}
        self.reasoner_id = None
        self.task_id = None
        self.task_name = ''
        self.par_names = []
        self.par_values = []

        # called by the deliberative tier..
        start_dialogue_task_service = rospy.Service(
            'start_dialogue_task', task_service, self.start_dialogue_task)

        # notifies the deliberative tier that a dialogue task has finished..
        self.dialogue_task_finished = rospy.ServiceProxy(
            'dialogue_task_finished', task_finished)

        # called by the gui for starting a dialogue by the user..
        listen_service = rospy.Service('listen', Trigger, self.listen)

        # called by any component that requires to start a dialogue..
        start_dialogue_service = rospy.Service(
            'start_dialogue', task_service, self.start_dialogue)

        # called by any component that requires a contextualized speech..
        contextualized_speech_service = rospy.Service(
            'contextualized_speech', task_service, self.contextualized_speech)

        # retrieves the current emotions..
        set_dialogue_parameters_service = rospy.Service(
            'set_dialogue_parameters', set_state, self.set_dialogue_parameters)

        # retrieves the current emotions..
        self.perceive_emotions = rospy.ServiceProxy(
            'perceive_emotions', get_state)

        # waits for the text to speech..
        self.text_to_speech = rospy.ServiceProxy(
            'text_to_speech', utterance_to_pronounce)
        self.text_to_speech.wait_for_service()

        # waits for the configuration of the speech to text..
        self.configure_speech_to_text = rospy.ServiceProxy(
            'configure_speech_to_text', Empty)
        self.configure_speech_to_text.wait_for_service()

        # waits for the speech to text..
        self.speech_to_text = rospy.ServiceProxy(
            'speech_to_text', utterance_to_recognize)
        self.speech_to_text.wait_for_service()

        # waits for the face manager..
        self.set_face = rospy.ServiceProxy(
            'set_face', face_to_show)
        self.set_face.wait_for_service()

        # waits for the image manager..
        self.show_image = rospy.ServiceProxy(
            'show_image', image_to_show)
        self.show_image.wait_for_service()

        # waits for the audio manager..
        self.play_audio = rospy.ServiceProxy(
            'play_audio', audio_to_play)
        self.play_audio.wait_for_service()

        # waits for the video manager..
        self.play_video = rospy.ServiceProxy(
            'play_video', video_to_play)
        self.play_video.wait_for_service()

        # waits for the page manager..
        self.show_page = rospy.ServiceProxy(
            'show_page', page_to_show)
        self.show_page.wait_for_service()

        # waits for the question manager..
        self.ask_question = rospy.ServiceProxy(
            'ask_question', question_to_ask)
        self.ask_question.wait_for_service()

        # publishes the state of the dialogue manager..
        self.state_pub = rospy.Publisher(
            'dialogue_state', dialogue_state, queue_size=10, latch=True)
        self.state_pub.publish(dialogue_state(dialogue_state.idle))
        self.set_face(face_idle)

    def start_dialogue_task(self, req):
        rospy.logdebug('Start dialogue task "%s" request..',
                       req.task.task_name)
        # we store the informations about the starting dialogue task..
        self.deliberative_task = True
        return self.start_dialogue(req)

    def start_dialogue(self, req):
        rospy.logdebug('Start dialogue "%s" request..', req.task.task_name)
        for i in range(len(req.task.par_names)):
            rospy.logdebug(
                req.task.par_names[i] + ': %s', req.task.par_values[i])
        # we set the command state at executing..
        try:
            r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                              'include_events': 'NONE'}, json={'event': 'slot', 'name': 'command_state', 'value': 'executing', 'timestamp': time.time()})
            assert r.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        # we store the informations about the starting dialogue..
        self.reasoner_id = req.task.reasoner_id
        self.task_id = req.task.task_id
        self.task_name = req.task.task_name
        self.par_names = req.task.par_names
        self.par_values = req.task.par_values
        return task_serviceResponse(True)

    def contextualized_speech(self, req):
        rospy.logdebug('Generating contextualized speech for "%s" intent..',
                       req.task.task_name)
        for i in range(len(req.task.par_names)):
            rospy.logdebug(
                req.task.par_names[i] + ': %s', req.task.par_values[i])

        # we prepare the request..
        payload = self.task_to_payload(req.task)

        # we make the request..
        rospy.logdebug(
            'Generating responses for "%s" intent..', req.task.task_name)
        try:
            r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                              '/trigger_intent', params={'include_events': 'NONE'}, json=payload)
            assert r.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        # we update the state..
        self.state_pub.publish(
            dialogue_state(dialogue_state.speaking))
        j_res = r.json()
        self.state = j_res['tracker']['slots']
        self.print_state()
        # we execute the retrieved actions..
        self.execute_actions(j_res['messages'])

        return task_serviceResponse(True)

    def set_dialogue_parameters(self, req):
        rospy.logdebug('Setting "%s" dialogue parameters..', req.name)
        for i in range(len(req.par_names)):
            rospy.logdebug(
                req.par_names[i] + ': %s', req.par_values[i])
            r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                'include_events': 'NONE'}, json={'event': 'slot', 'name': req.par_names[i], 'value': req.par_values[i], 'timestamp': time.time()})
            if r.status_code != requests.codes.ok:
                return set_stateResponse(False)
        return set_stateResponse(True)

    def listen(self, req):
        if self.task_name:
            return TriggerResponse(False, 'Already having a dialogue..')
        else:
            self.task_name = 'start_interaction'
            return TriggerResponse(True, 'Opening the microphone..')

    def start(self):
        try:
            rospy.loginfo('Restarting the dialogue engine..')
            r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
                'include_events': 'NONE'}, json={'sender': user, 'message': '/restart'})
            assert r.status_code == requests.codes.ok
            rospy.loginfo('Initializing the dialogue engine..')
            r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                              'include_events': 'NONE'}, json={'event': 'slot', 'name': 'coherent', 'value': coherent, 'timestamp': time.time()})
            assert r.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.task_name:
                # we update the state..
                self.state_pub.publish(
                    dialogue_state(dialogue_state.configuring))
                self.set_face(face_listening)
                # we configure the speech to text..
                try:
                    stt_conf = self.configure_speech_to_text()
                except rospy.ServiceException:
                    rospy.logerr('Speech to text configuration service call failed\n' +
                                 ''.join(traceback.format_stack()))
                self.set_face(face_idle)

                # we add the current perceived emotions..
                self.set_emotions()

                # we prepare the request..
                payload = self.task_to_payload(self)

                # we make the request..
                rospy.logdebug(
                    'Generating responses for task "%s"..', self.task_name)
                try:
                    r = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                                      '/trigger_intent', params={'include_events': 'NONE'}, json=payload)
                    assert r.status_code == requests.codes.ok
                except requests.exceptions.RequestException as e:
                    rospy.logerr('Rasa server call failed\n' +
                                 ''.join(traceback.format_stack()))
                    raise SystemExit(e)

                # we update the state..
                self.state_pub.publish(
                    dialogue_state(dialogue_state.speaking))
                j_res = r.json()
                self.state = j_res['tracker']['slots']
                self.print_state()
                # we execute the retrieved actions..
                self.execute_actions(j_res['messages'])

                # we start a dialogue..
                while not self.close_dialogue():
                    self.interact()
            rate.sleep()

    def interact(self):
        # we update the state..
        self.state_pub.publish(dialogue_state(dialogue_state.listening))
        self.set_face(face_listening)
        # we listen..
        while True:
            stt = self.speech_to_text()
            if stt.utterance == '':
                rospy.logwarn('Recognized empty string..')
                try:
                    stt_conf = self.configure_speech_to_text()
                except rospy.ServiceException:
                    rospy.logerr('Speech to text configuration service call failed\n' +
                                 ''.join(traceback.format_stack()))
            else:
                break

        # we set the current perceived emotions..
        self.set_emotions()

        # we make the request..
        rospy.logdebug(
            'Generating responses for utterance "%s"..', stt.utterance)
        try:
            r = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
                'include_events': 'NONE'}, json={'sender': user, 'message': stt.utterance})
            assert r.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        # we update the state..
        self.state_pub.publish(dialogue_state(dialogue_state.speaking))
        # we execute the retrieved actions..
        self.execute_actions(r.json())

        try:
            r = requests.get('http://' + host + ':' + port + '/conversations/' + user +
                             '/tracker', params={'include_events': 'NONE'})
            assert r.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        j_res = r.json()
        self.state = j_res['slots']
        self.print_state()

    def close_dialogue(self):
        if self.state['command_state'] == 'done' or self.state['command_state'] == 'failure':
            if self.deliberative_task:
                par_names = []
                par_values = []
                for s in self.state:
                    par_names.append(s)
                    par_values.append(str(self.state[s]))
                # we close the current task..
                if self.state['command_state'] == 'done':
                    # the task is closed with a success..
                    rospy.logdebug(
                        'Closing task "%s" with a success..', self.task_name)
                    self.dialogue_task_finished(task(
                        self.reasoner_id, self.task_id, self.task_name, par_names, par_values), True)
                elif self.state['command_state'] == 'failure':
                    # the task is closed with a failure..
                    rospy.logdebug(
                        'Closing task "%s" with a failure..', self.task_name)
                    self.dialogue_task_finished(task(
                        self.reasoner_id, self.task_id, self.task_name, par_names, par_values), False)
                self.deliberative_task = False
                self.reasoner_id = -1
                self.task_id = -1
                self.task_name = ''
                self.par_names.clear()
                self.par_values.clear()

            # we update the state..
            self.state_pub.publish(dialogue_state(dialogue_state.idle))
            self.set_face(face_idle)
            return True
        else:
            # we are still talking..
            return False

    def set_emotions(self):
        try:
            perceived_emotions = self.perceive_emotions()
            for i in range(len(perceived_emotions.par_names)):
                r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': perceived_emotions.par_names[i], 'value': perceived_emotions.par_values[i], 'timestamp': time.time()})
                assert r.status_code == requests.codes.ok
        except rospy.ServiceException:
            rospy.logwarn('Emotions detection service call failed\n' +
                          ''.join(traceback.format_stack()))
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

    def execute_actions(self, actions):
        try:
            for action in actions:
                if 'custom' in action:
                    if 'face' in action['custom']:
                        self.set_face(action['custom']['face'])
                    if 'image' in action['custom']:
                        self.show_image(
                            action['custom']['image']['src'], action['custom']['image']['alt'])
                    if 'audio' in action['custom']:
                        srcs = []
                        for src in action['custom']['audio']:
                            srcs.append(
                                audio(src['src'], src['type']))
                        self.play_audio(srcs)
                    if 'video' in action['custom']:
                        srcs = []
                        for src in action['custom']['video']:
                            srcs.append(
                                video(src['src'], src['type']))
                        self.play_video(srcs)
                    if 'page' in action['custom']:
                        self.show_page(
                            action['custom']['page']['src'], action['custom']['page']['title'])
                    if 'question' in action['custom']:
                        btns = []
                        for btn in action['custom']['question']:
                            btns.append(
                                button(btn['text'], btn['intent']))
                        self.ask_question(
                            action['custom']['question']['facial_expression'], action['custom']['question']['text'], btns)
                    if 'text' in action['custom']:
                        self.text_to_speech(action['custom']['text'])
                else:
                    self.set_face(face_talking)
                    self.text_to_speech(action['text'])
                self.set_face(face_idle)
        except rospy.ServiceException:
            rospy.logerr('Action execution failed\n' +
                         ''.join(traceback.format_stack()))
            # we update the state..
            self.state_pub.publish(
                dialogue_state(dialogue_state.idle))
            self.set_face(face_idle)

    def task_to_payload(self, task):
        payload = {'name': task.task_name}
        if task.par_names:
            entities = {}
            for i in range(len(task.par_names)):
                entities[task.par_names[i]] = task.par_values[i]
            payload['entities'] = entities
        return payload

    def print_state(self):
        for s in self.state:
            rospy.logdebug('"%s": "%s"', s, self.state[s])


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Dialogue Manager..')

    user = rospy.get_param('user')
    host = rospy.get_param('~host', 'localhost')
    port = str(rospy.get_param('~port', 5005))

    dm = dialogue_manager()
    dm.start()
