#!/usr/bin/env python3
import rospy
import os.path
import json
from persistence_manager.srv import get_state, get_stateResponse, set_state, set_stateResponse


class storage:

    def __init__(self):
        self.task = False

        load_service = rospy.Service(
            'load', get_state, self.load)

        dump_service = rospy.Service(
            'dump', set_state, self.dump)

    def load(self, req):
        rospy.logdebug('Loading "%s"..', req.name)

        if os.path.isfile(req.name):
            rospy.logdebug('File "%s" exists..', req.name)
            par_names = []
            par_values = []
            with open(req.name) as json_file:
                data = json.load(json_file)
                for s in data:
                    rospy.logdebug('"%s": "%s"', s, data[s])
                    par_names.append(s)
                    par_values.append(data[s])
            return get_stateResponse(par_names, par_values, True)
        else:
            rospy.logdebug('File "%s" does not exist..', req.name)
            return get_stateResponse([], [], False)

    def dump(self, req):
        rospy.logdebug('Dumping "%s"..', req.name)
        data = {}
        for i in range(len(req.par_names)):
            rospy.logdebug('"%s": "%s"', req.par_names[i], req.par_values[i])
            data[req.par_names[i]] = req.par_values[i]
        with open(req.name, 'w') as outfile:
            json.dump(data, outfile)

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('persistence_manager',
                    anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Persistence Manager..')

    pm = storage()
    pm.start()
