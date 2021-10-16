#!/usr/bin/env python3
import rospy
import rospkg
from flask import Flask, request, render_template, json
from flask_socketio import SocketIO, emit
from msgs.msg import timelines, dialogue_state
from msgs.srv import set_eyes, set_eyesResponse, set_mouth, set_mouthResponse
from std_srvs.srv import Trigger, TriggerRequest


rospack = rospkg.RosPack()
node_path = rospack.get_path('robot_gui')
app = Flask(__name__,
            template_folder=node_path + '/templates',
            static_folder=node_path + '/static')
app.debug = True
socketio = SocketIO(app)


@app.route('/')
def index_view():
    return render_template('index.html')


@app.route('/timelines')
def timelines_view():
    return render_template('timelines.html')


@socketio.on('/open_microphone')
def open_mic():
    try:
        res = open_microphone()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def emit_timelines(msg):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', msg)


def emit_dialogue_state(msg):
    if msg.dialogue_state == dialogue_state.listening:
        socketio.emit('microphone_state', {'state': 'Active'}, broadcast=True)
    else:
        socketio.emit('microphone_state', {
                      'state': 'Inactive'}, broadcast=True)


def emit_eyes(req):
    if req.eyes == set_eyes.default:
        socketio.emit('eyes_state', {
            'eyes': 'default'}, broadcast=True)
    return set_eyesResponse(True)


def emit_mouth(req):
    if req.mouth == set_mouth.default:
        socketio.emit('mouth_state', {
            'mouth': 'default'}, broadcast=True)
    return set_mouthResponse(True)


if __name__ == '__main__':
    rospy.init_node('robot_gui', anonymous=True)
    rospy.loginfo('Starting Robot GUI..')

    rospy.logdebug('Waiting for microphone activation service..')
    rospy.wait_for_service('open_microphone')
    open_microphone = rospy.ServiceProxy('open_microphone', Trigger)
    req = TriggerRequest()

    rospy.Subscriber('timelines', timelines, emit_timelines)
    rospy.Subscriber('dialogue_state', dialogue_state, emit_dialogue_state)

    show_eys_service = rospy.Service(
        'set_eyes', set_eyes, emit_eyes)
    show_mth_service = rospy.Service(
        'set_mouth', set_mouth, emit_mouth)

    socketio.run(app, host='localhost', port=8080)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')
