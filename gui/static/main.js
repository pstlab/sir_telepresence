const gui_host = 'localhost'
const gui_port = '8080'

const state = {
    sequencer_state: null,
    deliberative_state: new Map(),
    dialogue_state: null
};

const reasoners = new Map();

let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + gui_host + ':' + gui_port + '/solver');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) { }
    }
    ws.onclose = () => setTimeout(setup_ws, 1000);

    const response = await fetch('http://' + gui_host + ':' + gui_port + '/state');
    const state = await response.json();
    console.log(state);
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
    if (deliberative_state.size == 0) return '-';
    return Array.from(deliberative_state, ([id, state]) => {
        switch (state) {
            case 0: return '(' + id + ') idle'
            case 1: return '(' + id + ') reasoning'
            case 2: return '(' + id + ') executing'
            case 3: return '(' + id + ') finished'
            case 4: return '(' + id + ') inconsistent'
            default: return '(' + id + ') -';
        }
    }).join(', ');
}

function print_dialogue_state(dialogue_state) {
    switch (dialogue_state) {
        case 0: return 'idle'
        case 1: return 'configuring'
        case 2: return 'speaking'
        case 3: return 'listening'
        case 4: return 'waiting'
        default: return '-';
    }
}