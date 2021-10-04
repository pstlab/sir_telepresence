#!/usr/bin/env python3
import rospy
import rospkg
from flask import Flask, request, render_template, json
from flask_socketio import SocketIO, emit
from std_srvs.srv import Trigger, TriggerRequest
from msgs.msg import timelines, dialogue_state


rospack = rospkg.RosPack()
node_path = rospack.get_path('robot_gui')
app = Flask(__name__,
            template_folder=node_path + '/templates',
            static_folder=node_path + '/static')
app.debug = True
socketio = SocketIO(app)


def update_timelines(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)


@app.route('/')
def index_view():
    return render_template('index.html')


@app.route('/timelines')
def timelines_view():
    return render_template('timelines.html')


@socketio.on('/activate_microphone')
def activate_microphone():
    try:
        res = open_mic(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def update_dialogue_state(state):
    if state.dialogue_state == 2:
        socketio.emit('microphone_state', {'state': 'Active'}, broadcast=True)
    else:
        socketio.emit('microphone_state', {
                      'state': 'Inactive'}, broadcast=True)


if __name__ == '__main__':
    rospy.init_node('robot_gui', anonymous=True)
    rospy.loginfo('Starting Robot GUI..')

    rospy.logdebug('Waiting for microphone activation service..')
    rospy.wait_for_service('activate_microphone')
    open_mic = rospy.ServiceProxy('activate_microphone', Trigger)
    req = TriggerRequest()

    rospy.Subscriber('timelines', timelines, update_timelines)
    rospy.Subscriber('dialogue_state', dialogue_state, update_dialogue_state)

    socketio.run(app, host='localhost', port=8080)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')
