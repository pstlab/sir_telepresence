from typing import Text, List, Any, Dict

from rasa_sdk import Tracker, FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict


class ValidateCountTheWordForm(FormValidationAction):
    def name(self) -> Text:
        return 'validate_count_the_word_form'

    def validate_num_words(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        words = tracker.get_slot('word_sequence').split()
        num_words = words.count(tracker.get_slot('word'))
        if slot_value == num_words:
            dispatcher.utter_message(response='utter_positive_feedback')
            return {'num_words': slot_value}
        else:
            if tracker.get_slot('extraversion') == 'extroverted':
                dispatcher.utter_message(response='utter_negative_feedback_estroverse')
            else:
                dispatcher.utter_message(response='utter_negative_feedback_introverse')
            return {'num_words': None}