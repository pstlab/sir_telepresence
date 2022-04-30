#!/usr/bin/env python3
import rospy
import requests
import time
import dateutil.parser
from datetime import datetime
import pytz
import traceback
from std_srvs.srv import Trigger, TriggerResponse, Empty
from dialogue_manager.msg import dialogue_state, audio, video, button
from dialogue_manager.srv import utterance_to_pronounce, face_to_show, image_to_show, audio_to_play, video_to_play, page_to_show, question_to_ask, utterance_to_recognize, reminder_to_set
from deliberative_tier.msg import task
from deliberative_tier.srv import task_service, task_serviceResponse, task_finished
from persistence_manager.srv import get_state, set_state, set_stateRequest, set_stateResponse


#coherent = True
# the common faces..
face_idle = 'idle'
face_talking = 'talking'
face_listening = 'listening'


class dialogue_manager:

    def __init__(self):
        self.state = dialogue_state.idle
        self.slots = {}
        self.deliberative_task = False
        self.current_task = None

        # set the coherent param 
        self.coherent = rospy.get_param('coherent')
        self.actions_queue = []
        self.slots_updates_queue = []
        self.init_ros_services()
        self.set_dialogue_state(dialogue_state.idle, '')

    def init_ros_services(self):
        # called by the deliberative tier..
        self.start_dialogue_task_service = rospy.Service(
            'start_dialogue_task', task_service, self.start_dialogue_task)

        # called by any component that requires to start a dialogue..
        self.start_dialogue_service = rospy.Service(
            'start_dialogue', task_service, self.start_dialogue)

        # called by any component that requires a contextualized speech..
        self.contextualized_speech_service = rospy.Service(
            'contextualized_speech', task_service, self.blocking_contextualized_speech)

        # called by any component that requires a non-blocking contextualized speech..
        self.contextualized_speech_service = rospy.Service(
            'non_blocking_contextualized_speech', task_service, self.non_blocking_contextualized_speech)

        # called by the gui for starting a dialogue by the user..
        self.talk_to_me_service = rospy.Service(
            'talk_to_me', Trigger, self.talk_to_me)

        # sets the given dialogue parameters..
        self.set_dialogue_parameters_service = rospy.Service(
            'set_dialogue_parameters', set_state, self.set_dialogue_parameters)

        # notifies the deliberative tier that a dialogue task has finished..
        self.dialogue_task_finished = rospy.ServiceProxy(
            'task_finished', task_finished)

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

        # stores informations locally..
        self.dump = rospy.ServiceProxy(
            'dump', set_state)
        self.dump.wait_for_service()

        # loads informations locally..
        self.load = rospy.ServiceProxy(
            'load', get_state)
        self.load.wait_for_service()

        # waits for the question manager..
        self.ask_question = rospy.ServiceProxy(
            'ask_question', question_to_ask)
        self.ask_question.wait_for_service()

        # waits for reminder setting service..
        self.set_reminder = rospy.ServiceProxy(
            'set_reminder', reminder_to_set)
        self.set_reminder.wait_for_service()

        # publishes the state of the dialogue manager..
        self.state_pub = rospy.Publisher(
            'dialogue_state', dialogue_state, queue_size=10, latch=True)

    def start(self):
        # we reset the slots within the dialogue server..
        self.reset_dialogue()
        rate = rospy.Rate(50)
        # this is the main dialogue controller loop..
        while not rospy.is_shutdown():
            if self.actions_queue:
                self.execute_actions(self.actions_queue)
                self.actions_queue.clear()
            elif self.slots_updates_queue:
                for slot_update in self.slots_updates_queue:
                    self.update_slots(slot_update)
                self.slots_updates_queue.clear()
            elif self.state == dialogue_state.listening:
                while True:
                    stt = self.speech_to_text()
                    if stt.utterance == '':
                        rospy.logwarn('Recognized empty string..')
                        self.set_dialogue_state(dialogue_state.recognized, '')
                        try:
                            stt_conf = self.configure_speech_to_text()
                        except rospy.ServiceException:
                            rospy.logerr('Speech to text configuration service call failed\n' +
                                         ''.join(traceback.format_stack()))
                    else:
                        self.set_dialogue_state(
                            dialogue_state.recognized, stt.utterance)
                        break

                # we update the current perceived emotions..
                self.set_emotions()

                # we make the request..
                rospy.logdebug(
                    'Generating responses for utterance "%s"..', stt.utterance)
                self.parse_message(stt.utterance)
                try:
                    resp_req = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
                        'include_events': 'NONE'}, json={'sender': user, 'message': stt.utterance})
                    assert resp_req.status_code == requests.codes.ok
                    state_req = requests.get('http://' + host + ':' + port + '/conversations/' + user +
                                             '/tracker', params={'include_events': 'NONE'})
                    assert state_req.status_code == requests.codes.ok
                except requests.exceptions.RequestException as e:
                    rospy.logerr('Rasa server call failed\n' +
                                 ''.join(traceback.format_stack()))
                    raise SystemExit(e)

                # we enqueue the retrieved actions and update the internal state..
                for action in resp_req.json():
                    self.actions_queue.append(action)
                self.slots_updates_queue.append(state_req.json()['slots'])

            elif self.state == dialogue_state.idle and self.current_task is not None:
                # we configure the speech to text..
                self.configure()
                # we update the current perceived emotions..
                self.set_emotions()
                # we prepare the request..
                payload = self.task_to_payload(self.current_task)

                # we make the request..
                rospy.logdebug(
                    'Generating responses for task "%s"..', self.current_task.task_name)
                try:
                    intent_trg_req = requests.post('http://' + host + ':' + port + '/conversations/' + user +
                                                   '/trigger_intent', params={'include_events': 'NONE'}, json=payload)
                    assert intent_trg_req.status_code == requests.codes.ok
                except requests.exceptions.RequestException as e:
                    rospy.logerr('Rasa server call failed\n' +
                                 ''.join(traceback.format_stack()))
                    raise SystemExit(e)

                # we enqueue the retrieved actions and update the internal state..
                j_res = intent_trg_req.json()
                for action in j_res['messages']:
                    self.actions_queue.append(action)
                self.slots_updates_queue.append(j_res['tracker']['slots'])

            rate.sleep()

    def start_dialogue_task(self, req):
        assert self.current_task is None
        rospy.logdebug('Start dialogue task "%s" request..',
                       req.task.task_name)
        # we store the informations about the starting dialogue task..
        self.deliberative_task = True
        return self.start_dialogue(req)

    def start_dialogue(self, req):
        assert self.current_task is None
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

        # we store the informations about the current task..
        self.current_task = req.task
        return task_serviceResponse(True)

    def blocking_contextualized_speech(self, req):
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

        j_res = r.json()
        # we execute the retrieved actions and update the internal state..
        self.execute_actions(j_res['messages'])
        self.update_slots(j_res['tracker']['slots'])

        return task_serviceResponse(True)

    def non_blocking_contextualized_speech(self, req):
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

        # we enqueue the retrieved actions and update the internal state..
        j_res = r.json()
        for action in j_res['messages']:
            self.actions_queue.append(action)
        self.slots_updates_queue.append(j_res['tracker']['slots'])

        return task_serviceResponse(True)

    def talk_to_me(self, req):
        if self.current_task:
            return TriggerResponse(False, 'Already having a dialogue..')
        else:
            # we set the current task..
            self.current_task = task(None, None, 'start_interaction', [], [])
            # we set the command state at executing..
            try:
                r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                                  'include_events': 'NONE'}, json={'event': 'slot', 'name': 'command_state', 'value': 'executing', 'timestamp': time.time()})
                assert r.status_code == requests.codes.ok
            except requests.exceptions.RequestException as e:
                rospy.logerr('Rasa server call failed\n' +
                             ''.join(traceback.format_stack()))
                raise SystemExit(e)
            return TriggerResponse(True, 'Opening the microphone..')

    def set_dialogue_parameters(self, req):
        rospy.logdebug('Setting "%s" dialogue parameters..', req.name)
        try:
            for i in range(len(req.par_names)):
                rospy.logdebug(
                    req.par_names[i] + ': %s', req.par_values[i])
                r = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': req.par_names[i], 'value': req.par_values[i], 'timestamp': time.time()})
                assert r.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)
        return set_stateResponse(True)

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
        # we notify the state change..
        rospy.logdebug('Executing actions..')
        for action in actions:
            if 'custom' in action:
                if 'face' in action['custom'] and not 'answers' in action['custom']:
                    self.set_face(action['custom']['face'])
                if 'image' in action['custom']:
                    self.show_image(
                        action['custom']['image']['src'], action['custom']['image']['alt'])
                if 'audio' in action['custom']:
                    self.set_dialogue_state(dialogue_state.waiting, '')
                    srcs = []
                    for src in action['custom']['audio']:
                        srcs.append(
                            audio(src['src'], src['type']))
                    self.play_audio(srcs)
                if 'video' in action['custom']:
                    self.set_dialogue_state(dialogue_state.waiting, '')
                    srcs = []
                    for src in action['custom']['video']:
                        srcs.append(
                            video(src['src'], src['type']))
                    self.play_video(srcs)
                if 'page' in action['custom']:
                    self.set_dialogue_state(dialogue_state.waiting, '')
                    self.show_page(
                        action['custom']['page']['src'], action['custom']['page']['title'])
                if 'answers' in action['custom']:
                    self.set_dialogue_state(dialogue_state.waiting, '')
                    btns = []
                    for btn in action['custom']['answers']:
                        btns.append(
                            button(btn['text'], btn['intent']))
                    self.ask_question(
                        action['custom']['face'], action['custom']['text'], btns)
                if 'text' in action['custom']:
                    if not 'pronounce_text' in action['custom'] or action['custom']['pronounce_text']:
                        self.set_dialogue_state(
                            dialogue_state.speaking, action['custom']['text'])
                        self.text_to_speech(action['custom']['text'])
            else:
                self.set_dialogue_state(
                    dialogue_state.speaking, action['text'])
                self.set_face(face_talking)
                self.text_to_speech(action['text'])

    def update_slots(self, slots):
        self.slots = slots
        for s in self.slots:
            if self.slots[s] == 'None':
                self.slots[s] = None
            elif self.slots[s] == 'True':
                self.slots[s] = True
            elif self.slots[s] == 'False':
                self.slots[s] = False

        if self.slots['reminder_to_set_time'] is not None and self.slots['reminder_to_set_type'] is not None:
            rospy.logdebug('A new "%s" reminder, at time "%s", has been requested..',
                           self.slots['reminder_to_set_type'], self.slots['reminder_to_set_time'])
            # we set the reminder..
            request_time = datetime.now(tz=pytz.timezone('Europe/Rome'))
            reminder_time = dateutil.parser.parse(
                self.slots['reminder_to_set_time'], tzinfos={"CET": dateutil.tz.gettz("Europe/Rome")})
            waiting_time = int((reminder_time - request_time).total_seconds())
            rospy.logdebug('The "%s" reminder is about to be set in "%s" seconds from now..',
                           self.slots['reminder_to_set_type'], waiting_time)
            self.set_reminder(waiting_time, self.slots['reminder_to_set_type'])
            rospy.logdebug('The "%s" reminder has been set in "%s" seconds from now..',
                           self.slots['reminder_to_set_type'], waiting_time)
            # we clear the reminder to set..
            try:
                remove_reminder_time_req = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': 'reminder_to_set_time', 'value': None, 'timestamp': time.time()})
                assert remove_reminder_time_req.status_code == requests.codes.ok
                del self.slots['reminder_to_set_time']
                remove_reminder_type_req = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                    'include_events': 'NONE'}, json={'event': 'slot', 'name': 'reminder_to_set_type', 'value': None, 'timestamp': time.time()})
                assert remove_reminder_type_req.status_code == requests.codes.ok
                del self.slots['reminder_to_set_type']
            except requests.exceptions.RequestException as e:
                rospy.logerr('Rasa server call failed\n' +
                             ''.join(traceback.format_stack()))
                raise SystemExit(e)

        if self.slots['command_state'] == 'done' or self.slots['command_state'] == 'failure':
            rospy.logdebug('Closing dialogue..')
            self.print_story()
            if self.deliberative_task:
                rospy.logdebug('Closing "%s" dialogue..',
                               self.current_task.task_name)
                par_names = []
                par_values = []
                for s in self.slots:
                    par_names.append(s)
                    par_values.append(str(self.slots[s]))
                # we close the current task..
                if self.slots['command_state'] == 'done':
                    # the task is closed with a success..
                    rospy.logdebug(
                        'Closing "%s" dialogue with a success..', self.current_task.task_name)
                    if self.current_task.task_name == 'start_profile_gathering':
                        rospy.loginfo('Storing gathered profile..')
                        self.dump('profile.json', par_names, par_values)
                    self.dialogue_task_finished(task(
                        self.current_task.reasoner_id,
                        self.current_task.task_id,
                        self.current_task.task_name,
                        par_names,
                        par_values), True)
                elif self.slots['command_state'] == 'failure':
                    # the task is closed with a failure..
                    rospy.logdebug(
                        'Closing "%s" dialogue with a failure..', self.current_task.task_name)
                    self.dialogue_task_finished(task(
                        self.current_task.reasoner_id,
                        self.current_task.task_id,
                        self.current_task.task_name,
                        par_names,
                        par_values), False)
                self.deliberative_task = False
            self.current_task = None

            # we notify the state change..
            self.set_dialogue_state(dialogue_state.idle, '')
        elif self.state != dialogue_state.waiting:
            # we notify the state change..
            self.set_dialogue_state(dialogue_state.listening, '')

    def set_dialogue_state(self, state, text):
        self.state = state
        # we notify the state change..
        ds = dialogue_state(dialogue_state=state, text=text)
        ds.header.stamp = rospy.Time.now()
        self.state_pub.publish(ds)
        if state == dialogue_state.idle:
            self.set_face(face_idle)
        elif state == dialogue_state.configuring:
            self.set_face(face_listening)
        elif state == dialogue_state.listening:
            self.set_face(face_listening)

    def task_to_payload(self, task):
        payload = {'name': task.task_name}
        if task.par_names:
            entities = {}
            for i in range(len(task.par_names)):
                entities[task.par_names[i]] = task.par_values[i]
            payload['entities'] = entities
        return payload

    def reset_dialogue(self):
        try:
            rospy.loginfo('Resetting the dialogue engine..')
            intent_trg_req = requests.post('http://' + host + ':' + port + '/webhooks/rest/webhook', params={
                'include_events': 'NONE'}, json={'sender': user, 'message': '/restart'})
            assert intent_trg_req.status_code == requests.codes.ok

            rospy.loginfo('Initializing the dialogue engine..')
            intent_trg_req = requests.post('http://' + host + ':' + port + '/conversations/' + user + '/tracker/events', params={
                'include_events': 'NONE'}, json={'event': 'slot', 'name': 'coherent', 'value': self.coherent, 'timestamp': time.time()})
            assert intent_trg_req.status_code == requests.codes.ok

            rospy.loginfo('Checking for existing profile..')
            load_profile_call = self.load('profile.json')
            if load_profile_call.success:
                rospy.loginfo('Existing profile found!')
                for i in range(len(load_profile_call.par_names)):
                    if load_profile_call.par_values[i] == 'None':
                        load_profile_call.par_values[i] = None
                    elif load_profile_call.par_values[i] == 'True':
                        load_profile_call.par_values[i] = True
                    elif load_profile_call.par_values[i] == 'False':
                        load_profile_call.par_values[i] = False
                    rospy.loginfo(
                        ' - ' + load_profile_call.par_names[i] + ': %s', load_profile_call.par_values[i])
                self.set_dialogue_parameters(set_stateRequest(
                    'profile.json', load_profile_call.par_names, load_profile_call.par_values))
            else:
                rospy.loginfo('Existing profile not found..')
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

    def configure(self):
        self.set_dialogue_state(dialogue_state.configuring, '')
        try:
            stt_conf = self.configure_speech_to_text()
        except rospy.ServiceException:
            rospy.logerr('Speech to text configuration service call failed\n' +
                         ''.join(traceback.format_stack()))
        self.set_face(face_idle)

    def parse_message(self, message):
        try:
            parse_req = requests.post('http://' + host + ':' + port +
                                      '/model/parse', json={'text': message})
            assert parse_req.status_code == requests.codes.ok
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

        j_res = parse_req.json()
        rospy.logdebug(
            'Intent "%s" has been recognized with %f confidence..', j_res['intent']['name'], j_res['intent']['confidence'])

    def print_story(self):
        try:
            story_req = requests.get('http://' + host + ':' + port + '/conversations/' + user +
                                     '/story', params={'include_events': 'NONE'})
            assert story_req.status_code == requests.codes.ok
            print(story_req.text)
        except requests.exceptions.RequestException as e:
            rospy.logerr('Rasa server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)

    def print_state(self):
        rospy.logdebug('Dialogue state:')
        for s in self.state:
            rospy.logdebug(' - "%s": "%s"', s, self.state[s])


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Dialogue Manager..')

    user = rospy.get_param('user')
    host = rospy.get_param('~host', 'localhost')
    port = str(rospy.get_param('~port', 5005))

    dm = dialogue_manager()
    dm.start()
