from typing import Any, Text, Dict, List
from rasa_sdk.events import SlotSet
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
import paho.mqtt.client as mqtt
import logging


def on_connect(client, userdata, flags, rc):
    logging.info('Connected with result code ' + str(rc))


client = mqtt.Client('RASA Actions')
client.on_connect = on_connect
client.connect('localhost')
client.loop_start()


class ActionSetPrefix(Action):
    def name(self) -> Text:
        return 'action_mqtt_prefix'

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        n = tracker.get_latest_entity_values('number')
        try:
            prefix = str(next(n))
            while True:
                try:
                    prefix += '/' + str(next(n))
                except StopIteration:
                    break
            logging.info('MQTT prefix: ' + prefix)
            return [SlotSet('mqtt_prefix', prefix)]
        except StopIteration:
            return []


class ActionCommandStart(Action):
    def name(self) -> Text:
        return 'action_command_start'

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        sigma = next(tracker.get_latest_entity_values('number'), None)
        if sigma:
            sigma = str(sigma)
            logging.info('starting command sigma: ' + sigma)
            return [SlotSet('sigma', sigma)]
        return []


class ActionCommandDone(Action):
    def name(self) -> Text:
        return 'action_command_done'

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        prefix = tracker.get_slot('mqtt_prefix')
        sigma = tracker.get_slot('sigma')
        if prefix and sigma:
            logging.info('ending command sigma: ' + sigma)
            client.publish(prefix + '/done', '[' + sigma + ']')
            return [SlotSet('sigma', None)]
        return []


class ActionCommandFailure(Action):
    def name(self) -> Text:
        return 'action_command_failure'

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        prefix = tracker.get_slot('mqtt_prefix')
        sigma = tracker.get_slot('sigma')
        if prefix and sigma:
            logging.info('failing command sigma: ' + sigma)
            client.publish(prefix + '/failure', '[' + sigma + ']')
            return [SlotSet('sigma', None)]
        return []
