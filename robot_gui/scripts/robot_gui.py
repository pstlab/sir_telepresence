#!/usr/bin/env python3
import rospy
import rospkg
import traceback
from flask import Flask, render_template
from flask_socketio import SocketIO
from msgs.msg import timelines, dialogue_state
from msgs.srv import set_string, set_stringResponse
from std_srvs.srv import Empty


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
    print('opening microphone..')
    try:
        res = listen()
    except rospy.ServiceException:
        rospy.logerr('Service call failed\n' +
                     ''.join(traceback.format_stack()))


def emit_timelines(msg):
    rospy.logdebug('emitting timelines..')
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', msg)


def emit_dialogue_state(msg):
    rospy.logdebug('emitting dialogue state..')
    socketio.emit('dialogue_state', {
                  'state': msg.dialogue_state}, broadcast=True)


def emit_face(req):
    rospy.logdebug('emitting face..')
    socketio.emit('face_state', {'face': req.text}, broadcast=True)
    return set_stringResponse(True)


if __name__ == '__main__':
    rospy.init_node('robot_gui', anonymous=True)
    rospy.loginfo('Starting Robot GUI..')

    listen = rospy.ServiceProxy('listen', Empty)
    listen.wait_for_service()

    rospy.Subscriber('timelines', timelines, emit_timelines)
    rospy.Subscriber('dialogue_state', dialogue_state, emit_dialogue_state)

    show_eys_service = rospy.Service(
        'set_face', set_string, emit_face)

    gui_host = rospy.get_param('~host', 'localhost')
    gui_port = int(rospy.get_param('~port', '8080'))
    socketio.run(app, host=gui_host, port=gui_port)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')
