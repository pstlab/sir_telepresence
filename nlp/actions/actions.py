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


class BloodPressureAnalysis(Action):

    def name(self) -> Text:
        return "action_blood_pressure_analysis"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(
            response='utter_blood_pressure_recap')
        systolic_blood_pressure = tracker.get_slot('systolic_blood_pressure')
        diastolic_blood_pressure = tracker.get_slot('diastolic_blood_pressure')
        if systolic_blood_pressure < 140 and diastolic_blood_pressure < 85:
            dispatcher.utter_message(
                response='utter_blood_pressure_ok')
        if systolic_blood_pressure > 140:
            dispatcher.utter_message(
                response='utter_high_systolic_blood_pressure')
        if diastolic_blood_pressure > 140:
            dispatcher.utter_message(
                response='utter_high_diastolic_blood_pressure')
        return [SlotSet('command_state', 'failure')]
