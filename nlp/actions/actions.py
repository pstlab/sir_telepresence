from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker, FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict
from rasa_sdk.events import SlotSet
import requests
import random

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
        if tracker.get_slot('requested_slot') != 'user_name':
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
        if tracker.get_slot('requested_slot') != 'user_location':
            return {}

        dispatcher.utter_message(
            response='utter_location_comment', LOC=slot_value)
        dispatcher.utter_message(
            response='utter_location_comment_continuation', LOC=slot_value)
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
        if tracker.get_slot('requested_slot') != 'user_difficulty_express_feelings':
            return {}

        print("user's intent: " + tracker.get_intent_of_latest_message())
        if tracker.get_intent_of_latest_message() == 'uncertain':
            dispatcher.utter_message(
                response='utter_more_precise_answer')
            return {'user_difficulty_express_feelings': None}

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
        if tracker.get_slot('requested_slot') != 'user_exciting_life':
            return {}

        print("user's intent: " + tracker.get_intent_of_latest_message())
        if tracker.get_intent_of_latest_message() == 'uncertain':
            dispatcher.utter_message(
                response='utter_more_precise_answer')
            return {'user_exciting_life': None}

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
        if tracker.get_slot('requested_slot') != 'user_unease_unknown_people':
            return {}

        print("user's intent: " + tracker.get_intent_of_latest_message())
        if tracker.get_intent_of_latest_message() == 'uncertain':
            dispatcher.utter_message(
                response='utter_more_precise_answer')
            return {'user_unease_unknown_people': None}

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
        if tracker.get_slot('requested_slot') != 'user_unease_attention':
            return {}

        print("user's intent: " + tracker.get_intent_of_latest_message())
        if tracker.get_intent_of_latest_message() == 'uncertain':
            dispatcher.utter_message(
                response='utter_more_precise_answer')
            return {'user_unease_attention': None}

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
        if tracker.get_slot('requested_slot') != 'user_talks_lot':
            return {}

        print("user's intent: " + tracker.get_intent_of_latest_message())
        if tracker.get_intent_of_latest_message() == 'uncertain':
            dispatcher.utter_message(
                response='utter_more_precise_answer')
            return {'user_talks_lot': None}

        extraversion = analyze_profile(tracker)

        return {'user_talks_lot': slot_value, 'user_extraversion': extraversion}


class ActionAnalyzeProfile(Action):

    def name(self) -> Text:
        return 'action_analyze_profile'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet('user_extraversion', analyze_profile(tracker))]


def next_count_the_word(tracker: Tracker):
    word_sequences = tracker.get_slot(
        'count_the_word_word_sequences').split('; ')
    words = tracker.get_slot('count_the_word_words').split('; ')
    assert len(word_sequences) == len(words)

    print('Computing next count the word..')
    c_ws = word_sequences[0].split(', ')
    random.shuffle(c_ws)
    c_ws = ', '.join(c_ws)
    print('Word sequence is: ' + c_ws)
    print('Word is: ' + words[0])
    if len(word_sequences) > 1:
        return {'count_the_word_word_sequences': '; '.join(word_sequences[1:]),
                'count_the_word_words': '; '.join(words[1:]),
                'count_the_word_word_sequence': c_ws,
                'count_the_word_word': words[0]}
    else:
        return {'count_the_word_word_sequences': None,
                'count_the_word_words': None,
                'count_the_word_word_sequence': c_ws,
                'count_the_word_word': words[0]}


class ActionInitCountTheWord(Action):

    def name(self) -> Text:
        return 'action_init_count_the_word'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        slots = next_count_the_word(tracker)
        to_set = []
        for s in slots:
            to_set.append(SlotSet(s, slots[s]))
        return to_set


class ValidateCountTheWordForm(FormValidationAction):
    def name(self) -> Text:
        return 'validate_count_the_word_form'

    def validate_count_the_word_num_word(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        if tracker.get_slot('requested_slot') != 'count_the_word_num_word':
            return {}

        words = tracker.get_slot('count_the_word_word_sequence').split(', ')
        word = tracker.get_slot('count_the_word_word')
        extraversion = tracker.get_slot('user_extraversion')
        print('The word "{}" appears {} times in the list "{}"'.format(
            word, str(words.count(word)), ', '.join(words)))
        print("user's response: " + str(slot_value))

        if slot_value == words.count(word):
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_positive_feedback_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_positive_feedback_intro')
            if tracker.get_slot('count_the_word_word_sequences') is not None:
                print('another round..')
                dispatcher.utter_message(
                    response='utter_count_the_word_more_challenging')
                slots = next_count_the_word(tracker)
                dispatcher.utter_message(
                    response='utter_describe_count_the_word', count_the_word_word=slots['count_the_word_word'])
                dispatcher.utter_message(
                    response='utter_start_count_the_word', count_the_word_word_sequence=slots['count_the_word_word_sequence'])
                slots['count_the_word_num_word'] = None
                return slots
            else:
                print('last round..')
                return {'count_the_word_num_word': slot_value}
        else:
            if extraversion == 'extroverted':
                dispatcher.utter_message(
                    response='utter_negative_feedback_estro')
            elif extraversion == 'introverted':
                dispatcher.utter_message(
                    response='utter_negative_feedback_intro')

            dispatcher.utter_message(
                response='utter_try_again_count_the_word')
            return {'count_the_word_num_word': None}


class ValidateReminderForm(FormValidationAction):
    def name(self) -> Text:
        return 'validate_reminder_form'

    def validate_reminder_to_set_type(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        if tracker.get_slot('requested_slot') != 'reminder_to_set_type':
            return {}

        reminder_to_set_type = tracker.get_slot('reminder_to_set_type')
        if reminder_to_set_type != 'take_medicines' or reminder_to_set_type != 'go_to_the_doctor':
            return {'reminder_to_set_type': None}
        return {}


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
