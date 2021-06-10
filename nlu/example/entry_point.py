import requests
import time

# questo, probabilmente, va preso da un file di configurazione
sender = '4/8'


# per testare la connessione
version = requests.get('http://localhost:5005/version')
print(version.json())


print('Mqtt')
# per settare gli slot (in questo caso, il prefisso mqtt) - per settare più slot il json può anche essere una lista
#slot_set = requests.post(
#    'http://localhost:5005/conversations/' + sender + '/tracker/events', params={'include_events': 'NONE'}, json={'event': 'slot', 'name': 'mqtt_prefix', 'value': sender, 'timestamp': time.time()})
#print(slot_set.json())

print('Talking')
# per 'parlare' con il chatbot
answer = requests.post(
    'http://localhost:5005/webhooks/rest/webhook', params={'include_events': 'NONE'}, json={'sender': sender, 'message': 'ciao'})
print(answer.json())

print('Activity')
# per 'avviare' le attività
act = requests.post(
    'http://localhost:5005/conversations/' + sender + '/trigger_intent', params={'include_events': 'NONE'}, json={'name': 'chest_press_start_command', 'entities': {'num_press': 20, 'weight': 5}})
print(act.json())


print('Talking')
# per 'parlare' con il chatbot
answer = requests.post(
    'http://localhost:5005/webhooks/rest/webhook', params={'include_events': 'NONE'}, json={'sender': sender, 'message': 'no'})
print(answer.json())
