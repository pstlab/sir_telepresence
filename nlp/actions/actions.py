from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker, FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict
from rasa_sdk.events import SlotSet
import requests

import gender_guesser.detector as gender

intraversion_threshold = 0.5


def analyze_profile(tracker: Tracker):
    user_difficulty_express_feelings = tracker.get_slot(
        'user_difficulty_express_feelings')
    user_exciting_life = tracker.get_slot('user_exciting_life')
    user_unease_unknown_people = tracker.get_slot('user_unease_unknown_people')
    user_unease_attention = tracker.get_slot('user_unease_attention')

    extraversion = 0
    n_profile_pars_set = 0
    if user_difficulty_express_feelings is not None:
        n_profile_pars_set += 1
        if not user_difficulty_express_feelings:
            extraversion += 1
    if user_exciting_life is not None:
        n_profile_pars_set += 1
        if user_exciting_life == 'exciting':
            extraversion += 1
    if user_unease_unknown_people is not None:
        n_profile_pars_set += 1
        if not user_unease_unknown_people:
            extraversion += 1
    if user_unease_attention is not None:
        n_profile_pars_set += 1
        if not user_unease_attention:
            extraversion += 1

    if n_profile_pars_set > 0:
        extraversion /= n_profile_pars_set

    print('current extraversion is: ' + str(extraversion))

    if extraversion <= intraversion_threshold:
        extraversion = 'introverted'
    elif extraversion <= 1-intraversion_threshold:
        extraversion = 'neutral'
    else:
        extraversion = 'extroverted'

    print('user is currently detected as: ' + str(extraversion))
    return extraversion


class ValidateProfileForm(FormValidationAction):
    def name(self) -> Text:
        return 'validate_profile_form'

    def validate_user_name(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_name')
        print("user's name: " + slot_value)
        if not tracker.get_slot('requested_slot') == 'user_name':
            return {}

        gndr = gender.Detector().get_gender(slot_value, 'italy')
        print("user's detected gender: " + gndr)

        dispatcher.utter_message(response='utter_nice_to_meet', PER=slot_value)
        return {'user_name': slot_value, 'user_gender': gndr}

    def validate_user_location(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_location')
        print("user's location: " + slot_value)
        if not tracker.get_slot('requested_slot') == 'user_location':
            return {}

        dispatcher.utter_message(
            response='utter_location_comment', LOC=slot_value)
        return {'user_location': slot_value}

    def validate_user_difficulty_express_feelings(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_difficulty_express_feelings')
        print("user's response: " + str(slot_value))
        if not tracker.get_slot('requested_slot') == 'user_difficulty_express_feelings':
            return {}

        coherent = tracker.get_slot('coherent')
        extraversion = analyze_profile(tracker)

        if coherent:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_express_feelings_yes_comment_estro' if slot_value else 'utter_express_feelings_no_comment_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_express_feelings_yes_comment_intro' if slot_value else 'utter_express_feelings_no_comment_intro')
        else:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_express_feelings_yes_comment_intro' if slot_value else 'utter_express_feelings_no_comment_intro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_express_feelings_yes_comment_estro' if slot_value else 'utter_express_feelings_no_comment_estro')

        return {'user_difficulty_express_feelings': slot_value, 'user_extraversion': extraversion}

    def validate_user_exciting_life(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_exciting_life')
        print("user's response: " + str(slot_value))
        if not tracker.get_slot('requested_slot') == 'user_exciting_life':
            return {}

        coherent = tracker.get_slot('coherent')
        extraversion = analyze_profile(tracker)

        if coherent:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_exciting_life_exciting_comment_estro' if slot_value == 'exciting' else 'utter_exciting_life_boring_comment_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_exciting_life_exciting_comment_intro' if slot_value == 'exciting' else 'utter_exciting_life_boring_comment_intro')
        else:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_exciting_life_exciting_comment_intro' if slot_value == 'exciting' else 'utter_exciting_life_boring_comment_intro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_exciting_life_exciting_comment_estro' if slot_value == 'exciting' else 'utter_exciting_life_boring_comment_estro')

        return {'user_exciting_life': slot_value, 'user_extraversion': extraversion}

    def validate_user_unease_unknown_people(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_unease_unknown_people')
        print("user's response: " + str(slot_value))
        if not tracker.get_slot('requested_slot') == 'user_unease_unknown_people':
            return {}

        coherent = tracker.get_slot('coherent')
        extraversion = analyze_profile(tracker)

        if coherent:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_unknown_people_yes_comment_estro' if slot_value else 'utter_unknown_people_no_comment_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_unknown_people_yes_comment_intro' if slot_value else 'utter_unknown_people_no_comment_intro')
        else:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_unknown_people_yes_comment_intro' if slot_value else 'utter_unknown_people_no_comment_intro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_unknown_people_yes_comment_estro' if slot_value else 'utter_unknown_people_no_comment_estro')

        return {'user_unease_unknown_people': slot_value, 'user_extraversion': extraversion}

    def validate_user_unease_attention(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_unease_attention')
        print("user's response: " + str(slot_value))
        if not tracker.get_slot('requested_slot') == 'user_unease_attention':
            return {}

        coherent = tracker.get_slot('coherent')
        extraversion = analyze_profile(tracker)

        if coherent:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_unease_attention_yes_comment_estro' if slot_value else 'utter_unease_attention_no_comment_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_unease_attention_yes_comment_intro' if slot_value else 'utter_unease_attention_no_comment_intro')
        else:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_unease_attention_yes_comment_intro' if slot_value else 'utter_unease_attention_no_comment_intro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_unease_attention_yes_comment_estro' if slot_value else 'utter_unease_attention_no_comment_estro')

        return {'user_unease_attention': slot_value, 'user_extraversion': extraversion}

    def validate_user_talks_lot(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:

        print('validate_user_talks_lot')
        print("user's response: " + str(slot_value))
        if not tracker.get_slot('requested_slot') == 'user_talks_lot':
            return {}

        extraversion = analyze_profile(tracker)

        return {'user_talks_lot': slot_value, 'user_extraversion': extraversion}


class ActionAnalyzeProfile(Action):

    def name(self) -> Text:
        return 'action_analyze_profile'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('user_extraversion', analyze_profile(tracker))]


class ValidateCountTheWordForm(FormValidationAction):
    def name(self) -> Text:
        return 'validate_count_the_word_form'

    def validate_cognitive_exercise_num_words(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        words = tracker.get_slot('cognitive_exercise_word_sequence').split()
        num_words = words.count(tracker.get_slot('cognitive_exercise_word'))
        extraversion = tracker.get_slot('user_extraversion')

        if slot_value == num_words:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_positive_feedback_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_positive_feedback_intro')
            return {'cognitive_exercise_num_words': slot_value}
        else:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_negative_feedback_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_negative_feedback_intro')

            dispatcher.utter_message(response='utter_count_the_word')
            return {'cognitive_exercise_num_words': None}


class ActionWeatherAnalysis(Action):

    def name(self) -> Text:
        return 'action_weather_analysis'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        api = '510caddbf2d17028e6d14ac57d9fe6b9'
        url = 'http://api.openweathermap.org/data/2.5/weather?appid=' + api + '&units=metric'

        city = tracker.get_slot('user_location')
        if city is not None:
            url + '&q=' + city

        weather_data = requests.get(url).json()

        tracker.get_latest_entity_values('weather_field')
        if 'temperature' in listOfStrings:
            dispatcher.utter_message(
                response='utter_temperature', temperature=weather_data['temp'])
        if 'humidity' in listOfStrings:
            dispatcher.utter_message(
                response='utter_humidity', humidity=weather_data['humidity'])

        return []


class ActionCommandStart(Action):

    def name(self) -> Text:
        return 'action_command_start'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'executing')]


class ActionCommandDone(Action):

    def name(self) -> Text:
        return 'action_command_done'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'done')]


class ActionCommandFailure(Action):

    def name(self) -> Text:
        return 'action_command_failure'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('command_state', 'failure')]
