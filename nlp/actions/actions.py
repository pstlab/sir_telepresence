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


class ActionCommandPending(Action):

    def name(self) -> Text:
        return "action_command_pending"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'pending')]


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


class ActionProfileAnalysis(Action):

    def name(self) -> Text:
        return 'action_profile_analysis'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        profile_hobbies = tracker.get_slot('profile_hobbies')
        profile_talkative = tracker.get_slot('profile_talkative')
        profile_midst = tracker.get_slot('profile_midst')
        profile_meet = tracker.get_slot('profile_meet')
        profile_aloof = tracker.get_slot('profile_aloof')
        profile_revive = tracker.get_slot('profile_revive')
        profile_confident = tracker.get_slot('profile_confident')

        extraversion = 0
        n_profile_pars_set = 0
        if profile_hobbies is not None:
            n_profile_pars_set += 1
            if profile_hobbies:
                extraversion += 1
        if profile_talkative is not None:
            n_profile_pars_set += 1
            if profile_talkative:
                extraversion += 1
        if profile_midst is not None:
            n_profile_pars_set += 1
            if profile_midst:
                extraversion += 1
        if profile_meet is not None:
            n_profile_pars_set += 1
            if profile_meet:
                extraversion += 1
        if profile_aloof is not None:
            n_profile_pars_set += 1
            if not profile_aloof:
                extraversion += 1
        if profile_revive is not None:
            n_profile_pars_set += 1
            if profile_revive:
                extraversion += 1
        if profile_confident is not None:
            n_profile_pars_set += 1
            if profile_confident:
                extraversion += 1

        if n_profile_pars_set > 0:
            extraversion /= n_profile_pars_set

        if extraversion <= 0.4:
            SlotSet('extraversion', 'extroverted')
        elif extraversion <= 0.6:
            SlotSet('extraversion', 'neutral')
        else:
            SlotSet('extraversion', 'introverted')
        return []


class ActionBloodPressureAnalysis(Action):

    def name(self) -> Text:
        return "action_blood_pressure_analysis"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(
            response='utter_blood_pressure_recap')
        systolic_blood_pressure = int(
            tracker.get_slot('systolic_blood_pressure'))
        diastolic_blood_pressure = int(
            tracker.get_slot('diastolic_blood_pressure'))
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
        blood_saturation = int(tracker.get_slot('blood_saturation'))
        if blood_saturation > 90:
            dispatcher.utter_message(
                response='utter_blood_saturation_ok')
        else:
            dispatcher.utter_message(
                response='utter_low_blood_saturation')
        return []
