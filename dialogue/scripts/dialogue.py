#!/usr/bin/env python3
import rospy
from msgs.msg import dialogue_state
from msgs.srv import start_task, start_taskResponse


def start_dialogue(req):
    return start_taskResponse(True)


if __name__ == '__main__':
    rospy.init_node('dialogue_manager', anonymous=True)
    rospy.loginfo('Starting Dialogue Manager..')

    state = rospy.Publisher('dialogue_state', dialogue_state, queue_size=10)
    service = rospy.Service('start_dialogue', start_task, start_dialogue)

    rospy.spin()
