#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from home_automation.msg import home_automation_state


class home:

    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        if rospy.has_param('~mqtt_username'):
            self.client.username_pw_set(mqtt_username, mqtt_password)

        rospy.loginfo('Connecting MQTT to %s:%s', mqtt_host, str(mqtt_port))
        self.client.connect(mqtt_host, mqtt_port)

        # start MQTT loop..
        self.client.loop_start()

        # register shutdown callback..
        rospy.on_shutdown(self.client.disconnect)
        rospy.on_shutdown(self.client.loop_stop)

        # publishes the state of the home automation manager..
        self.state_pub = rospy.Publisher(
            'home_automation_state', home_automation_state, queue_size=10, latch=True)
        self.state_pub.publish(
            home_automation_state(home_automation_state.idle))

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo('MQTT connected with result code: %i..', rc)
        if rc == 0:
            rospy.loginfo('Subscribing to "#"')
            client.subscribe('#')

    def on_message(self, client, userdata, msg):
        rospy.loginfo('%s %s', msg.topic, str(msg.payload))

    def start(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('home_automation',
                    anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting Home Automation Manager..')

    mqtt_host = rospy.get_param('~mqtt_host', 'localhost')
    mqtt_port = rospy.get_param('~mqtt_port', 1883)
    if rospy.has_param('~mqtt_username'):
        mqtt_username = rospy.get_param('~mqtt_username')
        mqtt_password = rospy.get_param('~mqtt_password')

    hm = home()
    hm.start()
