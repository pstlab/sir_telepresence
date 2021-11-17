from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.events import SlotSet
from rasa_sdk.executor import CollectingDispatcher
import time


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


class ActionBloodPressureAnalysis(Action):

    def name(self) -> Text:
        return "action_blood_pressure_analysis"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(
            response='utter_blood_pressure_recap')
        systolic_blood_pressure = tracker.get_slot('systolic_blood_pressure')
        diastolic_blood_pressure = tracker.get_slot('diastolic_blood_pressure')
        if systolic_blood_pressure < 130 and diastolic_blood_pressure < 85:
            dispatcher.utter_message(
                response='utter_blood_pressure_ok')
        if systolic_blood_pressure > 130:
            dispatcher.utter_message(
                response='utter_high_systolic_blood_pressure')
        if diastolic_blood_pressure > 130:
            dispatcher.utter_message(
                response='utter_high_diastolic_blood_pressure')
        return []


class ActionBloodSaturationAnalysis(Action):

    def name(self) -> Text:
        return "action_blood_saturation_analysis"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(
            response='utter_blood_saturation_recap')
        blood_saturation = tracker.get_slot('blood_saturation')
        if blood_saturation > 90:
            dispatcher.utter_message(
                response='utter_blood_saturation_ok')
        else:
            dispatcher.utter_message(
                response='utter_low_blood_saturation')
        return []


class ActionSleep(Action):

    def name(self) -> Text:
        return "action_sleep"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        sleep_time = tracker.get_slot('sleep_time')
        print('sleeping for ' + str(sleep_time))
        time.sleep(sleep_time)
        return []
