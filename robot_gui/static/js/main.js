import * as options from './options.js'

var ros = new ROSLIB.Ros({ url: 'ws://' + options.ros_host + ':' + options.ros_port });

ros.on('connection', function () {
    console.log('Connected to websocket server..');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed..');
});

var set_face_service = new ROSLIB.Service({ ros: ros, name: '/set_face', serviceType: 'dialogue_manager/set_string' });
set_face_service.advertise(function (request, response) {
    console.log('Setting robot face:' + request.text);
    document.getElementById('robot_face').src = 'static/faces/' + request.text + '.gif';
    response['success'] = true;
    return true;
});

/*
Ohmni.setSpeechLanguage('it-IT');
var text_to_speech_service = new ROSLIB.Service({ ros: ros, name: '/text_to_speech', serviceType: 'dialogue_manager/set_string' });
text_to_speech_service.advertise(function (request, response) {
    console.log('Synthesizing:' + request.text);
    Ohmni.say(request.text);
    response['success'] = true;
    return true;
});
*/

var listen = new ROSLIB.Service({ ros: ros, name: '/listen', serviceType: 'std_srvs/Trigger' });

var state = {
    sequencer_state: null,
    deliberative_state: null,
    dialogue_state: null
};

var sequencer_state_listener = new ROSLIB.Topic({ ros: ros, name: '/sequencer_state', messageType: 'sequencer_tier/sequencer_state' });
sequencer_state_listener.subscribe(function (message) {
    state.sequencer_state = message.sequencer_state;
    print_state();
});

var deliberative_state_listener = new ROSLIB.Topic({ ros: ros, name: '/deliberative_state', messageType: 'deliberative_tier/deliberative_state' });
deliberative_state_listener.subscribe(function (message) {
    state.deliberative_state = { reasoner: message.reasoner_id, state: message.deliberative_state };
    print_state();
});

var dialogue_state_listener = new ROSLIB.Topic({ ros: ros, name: '/dialogue_state', messageType: 'dialogue_manager/dialogue_state' });
dialogue_state_listener.subscribe(function (message) {
    state.dialogue_state = message.dialogue_state;
    print_state();
});

var reasoner_created_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_created', messageType: 'std_msgs/UInt64' });
reasoner_created_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was created..');

    const reasoners_tabs = document.getElementById('reasoners-tabs');
    const reasoner_tab_template = document.getElementById('reasoner-tab-template');
    const reasoner_tab = reasoner_tab_template.content.cloneNode(true).querySelector('li');
    reasoner_tab.setAttribute('id', 'r' + message.data + '-tab');
    const reasoner_button = reasoner_tab.querySelector('button');
    if (!reasoners_tabs.hasChildNodes())
        reasoner_button.classList.add('active');
    reasoner_button.setAttribute('id', 'r' + message.data + '-but');
    reasoner_button.setAttribute('data-bs-target', '#r' + message.data);
    reasoner_button.setAttribute('aria-controls', 'r' + message.data);
    reasoner_button.append('Reasoner (' + message.data + ')');
    reasoners_tabs.append(reasoner_tab);

    const reasoners_content = document.getElementById('reasoners-content');
    const reasoner_template = document.getElementById('reasoner-content-template');
    const reasoner = reasoner_template.content.cloneNode(true).querySelector('div');
    if (!reasoners_content.hasChildNodes())
        reasoner.classList.add('show', 'active');
    reasoner.setAttribute('id', 'r' + message.data);
    reasoner.setAttribute('aria-labelledby', 'r' + message.data + '-but');
    reasoners_content.append(reasoner);
});

var reasoner_destroyed_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_destroyed', messageType: 'std_msgs/UInt64' });
reasoner_destroyed_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was destroyed..');

    document.getElementById('r' + message.data + '-tab').remove();
    document.getElementById('r' + message.data).remove();
});

var flaw_created_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_created', messageType: 'deliberative_tier/flaw_created' });
flaw_created_listener.subscribe(function (message) {
    console.log('flaw_created: ' + message);
});

var flaw_state_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_state_changed', messageType: 'deliberative_tier/flaw_state_changed' });
flaw_state_changed_listener.subscribe(function (message) {
    console.log('flaw_state_changed: ' + message);
});

var flaw_cost_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_cost_changed', messageType: 'deliberative_tier/flaw_cost_changed' });
flaw_cost_changed_listener.subscribe(function (message) {
    console.log('flaw_cost_changed: ' + message);
});

var flaw_position_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_position_changed', messageType: 'deliberative_tier/flaw_position_changed' });
flaw_position_changed_listener.subscribe(function (message) {
    console.log('flaw_position_changed: ' + message);
});

var current_flaw_listener = new ROSLIB.Topic({ ros: ros, name: '/current_flaw', messageType: 'deliberative_tier/current_flaw' });
current_flaw_listener.subscribe(function (message) {
    console.log('current_flaw: ' + message);
});

var resolver_created_listener = new ROSLIB.Topic({ ros: ros, name: '/resolver_created', messageType: 'deliberative_tier/resolver_created' });
resolver_created_listener.subscribe(function (message) {
    console.log('resolver_created: ' + message);
});

var resolver_state_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/resolver_state_changed', messageType: 'deliberative_tier/resolver_state_changed' });
resolver_state_changed_listener.subscribe(function (message) {
    console.log('resolver_state_changed: ' + message);
});

var current_resolver_listener = new ROSLIB.Topic({ ros: ros, name: '/current_resolver', messageType: 'deliberative_tier/current_resolver' });
current_resolver_listener.subscribe(function (message) {
    console.log('current_resolver: ' + message);
});

var causal_link_added_listener = new ROSLIB.Topic({ ros: ros, name: '/causal_link_added', messageType: 'deliberative_tier/causal_link_added' });
causal_link_added_listener.subscribe(function (message) {
    console.log('causal_link_added: ' + message);
});

export function start_listen() {
    listen.callService(new ROSLIB.ServiceRequest({}), function (result) {
        if (result.success)
            console.log('Opening microphone..');
        else
            console.log(result.message);
    });
}

function print_state() {
    console.log('System: ' + print_sequencer_state(state.sequencer_state) + ', Deliberative: ' + print_deliberative_state(state.deliberative_state) + ', Dialogue:' + print_dialogue_state(state.dialogue_state));
}

function print_sequencer_state(sequencer_state) {
    switch (sequencer_state) {
        case 0: return 'unconfigured'
        case 1: return 'configuring'
        case 2: return 'configured'
        case 3: return 'running'
        default: return '-';
    }
}

function print_deliberative_state(deliberative_state) {
    if (deliberative_state)
        switch (deliberative_state.state) {
            case 0: return '(' + deliberative_state.reasoner + ') idle'
            case 1: return '(' + deliberative_state.reasoner + ') reasoning'
            case 2: return '(' + deliberative_state.reasoner + ') executing'
            case 3: return '(' + deliberative_state.reasoner + ') finished'
            case 4: return '(' + deliberative_state.reasoner + ') inconsistent'
            default: return '(' + deliberative_state.reasoner + ') -';
        }
    else return '-';
}

function print_dialogue_state(dialogue_state) {
    switch (dialogue_state) {
        case 0: return 'idle'
        case 1: return 'configuring'
        case 2: return 'speaking'
        case 3: return 'listening'
        default: return '-';
    }
}
