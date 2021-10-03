#!/usr/bin/env python3
from logging import debug
import rospy
import rospkg
# import waitress
from flask import Flask, request, render_template, json
from msgs.msg import timelines


rospack = rospkg.RosPack()
template_path = rospack.get_path('robot_gui') + '/templates'
app = Flask(__name__, template_folder=template_path)
app.debug = True


def update_timelines(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)


@app.route('/')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    rospy.init_node('robot_gui', anonymous=True)
    rospy.loginfo('Starting Robot GUI..')

    rospy.Subscriber('timelines', timelines, update_timelines)

    from waitress import serve
    serve(app, host='0.0.0.0', port=8080)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down..')
