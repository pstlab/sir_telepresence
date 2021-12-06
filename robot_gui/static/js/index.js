document.getElementById('home-nav-item').classList.add('active');
document.getElementById('timelines-nav-item').classList.remove('active');

var ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

ros.on('connection', function () {
    console.log('Connected to websocket server..');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed..');
});

var set_face_service = new ROSLIB.Service({ ros: ros, name: '/set_face', serviceType: 'msgs/set_string' });
set_face_service.advertise(function (request, response) {
    console.log('Setting robot face:' + request.text);
    document.getElementById('robot_face').src = 'static/faces/' + request.text + '.gif';
    response['success'] = true;
    return true;
});

/*
Ohmni.setSpeechLanguage('it-IT');
var text_to_speech_service = new ROSLIB.Service({ ros: ros, name: '/text_to_speech', serviceType: 'msgs/set_string' });
text_to_speech_service.advertise(function (request, response) {
    console.log('Synthesizing:' + request.text);
    Ohmni.say(request.text);
    response['success'] = true;
    return true;
});
*/

var wait_for_input_service = new ROSLIB.Service({ ros: ros, name: '/wait_for_input', serviceType: 'msgs/set_string' });
wait_for_input_service.advertise(function (request, response) {
    console.log('Waiting for input:' + request.text);
    response['success'] = true;
    return true;
});

var listen = new ROSLIB.Service({ ros: ros, name: '/listen', serviceType: 'std_srvs/Trigger' });

var state = {
    sequencer_state: null,
    deliberative_state: null,
    navigation_state: null,
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

var navigation_state_listener = new ROSLIB.Topic({ ros: ros, name: '/navigation_state', messageType: 'msgs/navigation_state' });
navigation_state_listener.subscribe(function (message) {
    state.navigation_state = message.navigation_state;
    print_state();
});

var dialogue_state_listener = new ROSLIB.Topic({ ros: ros, name: '/dialogue_state', messageType: 'dialogue/dialogue_state' });
dialogue_state_listener.subscribe(function (message) {
    state.dialogue_state = message.dialogue_state;
    print_state();
});

function start_listen() {
    listen.callService(new ROSLIB.ServiceRequest({}), function (result) {
        if (result.success)
            console.log('Opening microphone..');
        else
            console.log(result.message);
    });
}

function print_state() {
    console.log('System: ' + print_sequencer_state(state.sequencer_state) + ', Deliberative: ' + print_deliberative_state(state.deliberative_state) + ', Navigation: ' + print_navigation_state(state.navigation_state) + ', Dialogue:' + print_dialogue_state(state.dialogue_state));
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

function print_navigation_state(navigation_state) {
    switch (navigation_state) {
        case 0: return 'idle'
        case 1: return 'navigating'
        default: return '-';
    }
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
