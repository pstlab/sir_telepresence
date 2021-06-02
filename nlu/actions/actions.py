from typing import Any, Text, Dict, List
from rasa_sdk.events import SlotSet
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
import logging


class ActionCommandStart(Action):
    def name(self) -> Text:
        return 'action_command_start'

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        command_id = next(tracker.get_latest_entity_values('number'), None)
        logging.info('starting command id: ' + str(command_id))
        return [SlotSet('command_id', command_id)]


class ActionCommandDone(Action):
    def name(self) -> Text:
        return 'action_command_done'

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        command_id = tracker.get_slot('command_id')
        logging.info('ending command id: ' + str(command_id))
        return []
