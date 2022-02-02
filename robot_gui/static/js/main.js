import { Graph, GraphData } from './modules/graph.js';
import { Timelines, TimelinesData } from './modules/timelines.js';
import * as options from './options.js'

const navbar_height = document.getElementById('nav-bar').offsetHeight;
const reasoners_tab_height = 42;
const c_height = (window.innerHeight - navbar_height - reasoners_tab_height) / 2;

const timelines_data = new Map();
const timelines_chart = new Map();
const graph_data = new Map();
const graph = new Map();

const state = {
    sequencer_state: null,
    deliberative_state: null,
    dialogue_state: null
};

const ros = new ROSLIB.Ros({ url: 'ws://' + options.ros_host + ':' + options.ros_port });

ros.on('connection', function () {
    console.log('Connected to websocket server..');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed..');
});

const set_face_service = new ROSLIB.Service({ ros: ros, name: '/set_face', serviceType: 'dialogue_manager/face_to_show' });
set_face_service.advertise(function (request, response) {
    console.log('Setting robot face:' + request.facial_expression);
    document.getElementById('robot_face').src = 'static/faces/' + request.facial_expression + '.gif';
    document.getElementById('robot_face').height = 700
    document.getElementById('question').classList.add('d-none');
    response['success'] = true;
    return true;
});

const ask_question_service = new ROSLIB.Service({ ros: ros, name: '/ask_question', serviceType: 'dialogue_manager/question_to_ask' });
ask_question_service.advertise(function (request, response) {
    console.log('Asking question:' + request.text);
    document.getElementById('robot_face').src = 'static/faces/' + request.facial_expression + '.gif';
    document.getElementById('robot_face').height = 300
    document.getElementById('question').classList.remove('d-none');
    document.getElementById('question_text').innerText = request.text
    let buttons = [];
    request.buttons.forEach(button => {
        let btn = document.createElement('button');
        btn.classList.add('btn', 'btn-primary', 'btn-lg');
        btn.type = 'button';
        btn.innerHTML = button.text;
        buttons.push(btn);
    });
    document.getElementById('question_buttons').replaceChildren(buttons);

    response['success'] = true;
    return true;
});

/*
Ohmni.setSpeechLanguage('it-IT');
const text_to_speech_service = new ROSLIB.Service({ ros: ros, name: '/text_to_speech', serviceType: 'dialogue_manager/utterance_to_pronounce' });
text_to_speech_service.advertise(function (request, response) {
    console.log('Synthesizing:' + request.utterance);
    Ohmni.say(request.utterance);
    response['success'] = true;
    return true;
});
*/

const listen = new ROSLIB.Service({ ros: ros, name: '/listen', serviceType: 'std_srvs/Trigger' });

const sequencer_state_listener = new ROSLIB.Topic({ ros: ros, name: '/sequencer_state', messageType: 'sequencer_tier/sequencer_state' });
sequencer_state_listener.subscribe(function (message) {
    state.sequencer_state = message.sequencer_state;
    print_state();
});

const deliberative_state_listener = new ROSLIB.Topic({ ros: ros, name: '/deliberative_state', messageType: 'deliberative_tier/deliberative_state' });
deliberative_state_listener.subscribe(function (message) {
    state.deliberative_state = { reasoner: message.reasoner_id, state: message.deliberative_state };
    print_state();
});

const dialogue_state_listener = new ROSLIB.Topic({ ros: ros, name: '/dialogue_state', messageType: 'dialogue_manager/dialogue_state' });
dialogue_state_listener.subscribe(function (message) {
    state.dialogue_state = message.dialogue_state;
    print_state();
});

const reasoner_created_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_created', messageType: 'std_msgs/UInt64' });
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

    const timelines_div = document.createElement('div');
    timelines_div.setAttribute('id', 'r' + message.data + '-timelines');
    timelines_div.style.backgroundColor = 'white';
    reasoner.appendChild(timelines_div);
    timelines_data.set(message.data, new TimelinesData());
    timelines_chart.set(message.data, new Timelines('r' + message.data + '-timelines', window.innerWidth, c_height));

    const graph_div = document.createElement('div');
    graph_div.setAttribute('id', 'r' + message.data + '-graph');
    graph_div.style.backgroundColor = 'white';
    reasoner.appendChild(graph_div);
    graph_data.set(message.data, new GraphData());
    graph.set(message.data, new Graph('r' + message.data + '-graph', window.innerWidth, c_height));
});

const reasoner_destroyed_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_destroyed', messageType: 'std_msgs/UInt64' });
reasoner_destroyed_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was destroyed..');

    document.getElementById('r' + message.data + '-tab').remove();
    document.getElementById('r' + message.data).remove();

    timelines_data.delete(message.data);
    timelines_chart.delete(message.data);
    graph_data.delete(message.data);
    graph.delete(message.data);
});

const graph_listener = new ROSLIB.Topic({ ros: ros, name: '/graph', messageType: 'deliberative_tier/graph' });
graph_listener.subscribe(function (message) {
    switch (message.update) {
        case 0: // graph_changed
            graph_data.get(message.reasoner_id).reset(message);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 1: // flaw_created
            message.flaws[0].label = JSON.parse(message.flaws[0].label);
            graph_data.get(message.reasoner_id).flaw_created(message.flaws[0]);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 2: // flaw_state_changed
            graph_data.get(message.reasoner_id).flaw_state_changed(message.flaws[0]);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 3: // flaw_cost_changed
            graph_data.get(message.reasoner_id).flaw_cost_changed(message.flaws[0]);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 4: // flaw_position_changed
            graph_data.get(message.reasoner_id).flaw_position_changed(message.flaws[0]);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 5: // current_flaw
            graph_data.get(message.reasoner_id).current_flaw(message);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 6: // resolver_created
            message.resolvers[0].label = JSON.parse(message.resolvers[0].label);
            graph_data.get(message.reasoner_id).resolver_created(message.resolvers[0]);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 7: // resolver_state_changed
            graph_data.get(message.reasoner_id).resolver_state_changed(message.resolvers[0]);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 8: // current_resolver
            graph_data.get(message.reasoner_id).current_resolver(message);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        case 9: // causal_link_added
            graph_data.get(message.reasoner_id).causal_link_added(message);
            graph.get(message.reasoner_id).update(graph_data.get(message.reasoner_id));
            break;
        default:
            break;
    }
});

const timelines_listener = new ROSLIB.Topic({ ros: ros, name: '/timelines', messageType: 'deliberative_tier/timelines' });
timelines_listener.subscribe(function (message) {
    console.log(message);
    switch (message.update) {
        case 0: // timelines_time_changed
            message.timelines.forEach((tl, i) => message.timelines[i] = JSON.parse(tl));
            timelines_data.get(message.reasoner_id).reset(message);
            timelines_data.get(message.reasoner_id).tick(message.time);
            timelines_chart.get(message.reasoner_id).update(timelines_data.get(message.reasoner_id));
            timelines_chart.get(message.reasoner_id).updateTime(timelines_data.get(message.reasoner_id));
            break;
        case 1: // timelines_changed
            message.timelines.forEach((tl, i) => message.timelines[i] = JSON.parse(tl));
            timelines_data.get(message.reasoner_id).reset(message);
            timelines_chart.get(message.reasoner_id).update(timelines_data.get(message.reasoner_id));
            break;
        case 2: // time_changed
            timelines_data.get(message.reasoner_id).tick(message.time);
            timelines_chart.get(message.reasoner_id).updateTime(timelines_data.get(message.reasoner_id));
            break;
        default:
            break;
    }
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
