from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.events import SlotSet
from rasa_sdk.executor import CollectingDispatcher


class ActionCommandStart(Action):

    def name(self) -> Text:
        return "action_command_start"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'executing')]


class ActionCommandDone(Action):

    def name(self) -> Text:
        return "action_command_done"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'done')]


class ActionCommandFailure(Action):

    def name(self) -> Text:
        return "action_command_failure"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'failure')]
