const gui_host = 'localhost'
const gui_port = '8080'

var synth = window.speechSynthesis;
var voice = null;

if (speechSynthesis.onvoiceschanged !== undefined) {
    speechSynthesis.onvoiceschanged = function () {
        var voices = synth.getVoices();
        for (i = 0; i < voices.length; i++) {
            if (voices[i].name === 'Google italiano') {
                voice = voices[i];
                break;
            }
        }
    };
}

var SpeechRecognition = window.SpeechRecognition || webkitSpeechRecognition;
var recognition = new SpeechRecognition();
recognition.continuous = true;
recognition.lang = 'it-IT';
recognition.interimResults = true;
recognition.maxAlternatives = 1;
recognition.start();
recognition.onresult = function (event) {
    for (var i = event.resultIndex; i < event.results.length; ++i) {
        if (event.results[i].isFinal) {
            console.log('Final transcript: ' + event.results[i][0].transcript);
            console.log('Final confidence: ' + event.results[i][0].confidence);

            var utter = new SpeechSynthesisUtterance(event.results[i][0].transcript);
            utter.onend = function (event) { console.log('SpeechSynthesisUtterance.onend'); }
            utter.onerror = function (event) { console.error('SpeechSynthesisUtterance.onerror'); }
            utter.voice = voice;
            synth.speak(utter);
        } else {
            console.log('Interim transcript: ' + event.results[i][0].transcript);
            console.log('Interim confidence: ' + event.results[i][0].confidence);
        }
    }
}
recognition.onspeechend = function () {
    recognition.stop();
}

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
    ws.onopen = () => {
        const reasoners_tabs = document.getElementById('reasoners-tabs');
        while (reasoners_tabs.firstChild)
            reasoners_tabs.removeChild(reasoners_tabs.lastChild);
        const reasoners_content = document.getElementById('reasoners-content');
        while (reasoners_content.firstChild)
            reasoners_content.removeChild(reasoners_content.lastChild);

        document.getElementById('listen-button').onclick = (event) => ws.send(JSON.stringify({ type: 'talk_to_me' }));
    };
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'deliberative_state':
                switch (c_msg.state) {
                    case 0: // created
                        create_reasoner(c_msg.reasoner_id);
                        break;
                    case 1: // destroyed
                        document.getElementById('r' + c_msg.reasoner_id + '-tab').remove();
                        document.getElementById('r' + c_msg.reasoner_id).remove();
                        reasoners.delete(c_msg.reasoner_id);
                        break;
                    case 5: // finished
                    case 6: // inconsistent
                        {
                            const r = reasoners.get(c_msg.reasoner_id);
                            r.current_flaw = undefined;
                            r.current_resolver = undefined;
                            r.network.unselectAll();
                        }
                    case 2: // idle
                    case 3: // reasoning
                    case 4: // executing
                        state.deliberative_state.set(c_msg.reasoner_id, c_msg.state);
                        print_state();
                    default:
                        break;
                }
                break;
            case 'dialogue_state':
                state.dialogue_state = c_msg.state;
                document.getElementById('listen-button').disabled = state.dialogue_state != 0;
                print_state();
                break;
            case 'sequencer_state':
                state.sequencer_state = c_msg.state;
                print_state();
                break;
            case 'show_face':
                console.log('Setting robot face:' + c_msg.facial_expression);
                document.getElementById('face_div').classList.remove('d-none');
                document.getElementById('image_div').classList.add('d-none');
                document.getElementById('audio_div').classList.add('d-none');
                document.getElementById('video_div').classList.add('d-none');
                document.getElementById('html_div').classList.add('d-none');
                document.getElementById('question_div').classList.add('d-none');

                document.getElementById('robot_face').src = 'static/faces/' + c_msg.facial_expression + '.gif';
                document.getElementById('robot_face').height = 700
                break;
            case 'show_image':
                console.log('Showing image:' + c_msg.src);
                document.getElementById('face_div').classList.add('d-none');
                document.getElementById('image_div').classList.remove('d-none');
                document.getElementById('audio_div').classList.add('d-none');
                document.getElementById('video_div').classList.add('d-none');
                document.getElementById('html_div').classList.add('d-none');
                document.getElementById('question_div').classList.add('d-none');

                document.getElementById('image').src = c_msg.src;
                document.getElementById('image').alt = c_msg.alt;
                break;
            case 'play_audio':
                {
                    console.log('Playing audio:' + c_msg.src);
                    document.getElementById('face_div').classList.add('d-none');
                    document.getElementById('image_div').classList.add('d-none');
                    document.getElementById('audio_div').classList.remove('d-none');
                    document.getElementById('video_div').classList.add('d-none');
                    document.getElementById('html_div').classList.add('d-none');
                    document.getElementById('question_div').classList.add('d-none');

                    const audio = document.getElementById('audio');
                    let sources = [];
                    c_msg.videos.forEach(src => {
                        let source = document.createElement('source');
                        source.src = src.src;
                        source.type = src.type;
                        sources.push(source);
                    });
                    audio.replaceChildren(sources);

                    audio.play();
                }
                break;
            case 'play_video':
                {
                    console.log('Playing video:' + c_msg.src);
                    document.getElementById('face_div').classList.add('d-none');
                    document.getElementById('image_div').classList.add('d-none');
                    document.getElementById('audio_div').classList.add('d-none');
                    document.getElementById('video_div').classList.remove('d-none');
                    document.getElementById('html_div').classList.add('d-none');
                    document.getElementById('question_div').classList.add('d-none');

                    const video = document.getElementById('video');
                    let sources = [];
                    c_msg.videos.forEach(src => {
                        let source = document.createElement('source');
                        source.src = src.src;
                        source.type = src.type;
                        sources.push(source);
                    });
                    video.replaceChildren(sources);

                    video.play();
                }
                break;
            case 'show_page':
                console.log('Showing page:' + c_msg.src);
                document.getElementById('face_div').classList.add('d-none');
                document.getElementById('image_div').classList.add('d-none');
                document.getElementById('audio_div').classList.add('d-none');
                document.getElementById('video_div').classList.add('d-none');
                document.getElementById('html_div').classList.remove('d-none');
                document.getElementById('question_div').classList.add('d-none');

                document.getElementById('html').src = c_msg.src;
                document.getElementById('html').title = c_msg.title;
                break;
            case 'ask_question':
                {
                    console.log('Asking question:' + c_msg.text);
                    document.getElementById('face_div').classList.remove('d-none');
                    document.getElementById('image_div').classList.add('d-none');
                    document.getElementById('audio_div').classList.add('d-none');
                    document.getElementById('video_div').classList.add('d-none');
                    document.getElementById('html_div').classList.add('d-none');
                    document.getElementById('question_div').classList.remove('d-none');

                    document.getElementById('robot_face').src = 'static/faces/' + c_msg.facial_expression + '.gif';
                    document.getElementById('robot_face').height = 400
                    document.getElementById('question_text').innerText = c_msg.text

                    const question_buttons = document.getElementById('question_buttons');
                    while (question_buttons.firstChild)
                        question_buttons.removeChild(question_buttons.lastChild);
                    c_msg.buttons.forEach(button => {
                        let btn = document.createElement('button');
                        btn.classList.add('btn', 'btn-secondary', 'btn-lg');
                        btn.type = 'button';
                        btn.appendChild(document.createTextNode(button.text));
                        btn.innerHTML = button.text;
                        btn.onclick = (event) => ws.send(JSON.stringify({ type: 'answer_question', intent: button.intent }));
                        question_buttons.appendChild(btn);
                    });
                }
                break;
            case 'show_toast':
                {
                    console.log('Showing toast:' + c_msg.text);
                    const toast_div = document.getElementById('recognized_speech');
                    const toast_text = document.getElementById('recognized_speech_text');
                    toast_text.innerHTML = c_msg.text;
                    var toast = new bootstrap.Toast(toast_div)
                    toast.show()
                }
                break;
            case 'graph_changed':
                reasoners.get(c_msg.reasoner_id).graph(c_msg);
                break;
            case 'flaw_created':
                reasoners.get(c_msg.reasoner_id).flaw_created(c_msg);
                break;
            case 'flaw_state_changed':
                reasoners.get(c_msg.reasoner_id).flaw_state_changed(c_msg);
                break;
            case 'flaw_cost_changed':
                reasoners.get(c_msg.reasoner_id).flaw_cost_changed(c_msg);
                break;
            case 'flaw_position_changed':
                reasoners.get(c_msg.reasoner_id).flaw_position_changed(c_msg);
                break;
            case 'current_flaw':
                c_msg.id = c_msg.flaw_id;
                reasoners.get(c_msg.reasoner_id).current_flaw_changed(c_msg);
                break;
            case 'resolver_created':
                reasoners.get(c_msg.reasoner_id).resolver_created(c_msg);
                break;
            case 'resolver_state_changed':
                reasoners.get(c_msg.reasoner_id).resolver_state_changed(c_msg);
                break;
            case 'current_resolver':
                c_msg.id = c_msg.resolver_id;
                reasoners.get(c_msg.reasoner_id).current_resolver_changed(c_msg);
                break;
            case 'causal_link_added':
                reasoners.get(c_msg.reasoner_id).causal_link_added(c_msg);
                break;
            case 'state_changed':
                reasoners.get(c_msg.reasoner_id).state_changed(c_msg);
                break;
            case 'time_changed':
                reasoners.get(c_msg.reasoner_id).tick(c_msg);
                break;
            case 'executing_changed':
                reasoners.get(c_msg.reasoner_id).executing_changed(c_msg);
                break;
            default:
                console.log(c_msg.type);
        }
    }
    ws.onclose = () => setTimeout(setup_ws, 1000);
}

function create_reasoner(r_id) {
    const reasoners_tabs = document.getElementById('reasoners-tabs');
    const reasoner_tab_template = document.getElementById('reasoner-tab-template');
    const reasoner_tab = reasoner_tab_template.content.cloneNode(true).querySelector('li');
    reasoner_tab.setAttribute('id', 'r' + r_id + '-tab');
    const reasoner_button = reasoner_tab.querySelector('button');
    if (!reasoners_tabs.hasChildNodes())
        reasoner_button.classList.add('active');
    reasoner_button.setAttribute('id', 'r' + r_id + '-but');
    reasoner_button.setAttribute('data-bs-target', '#r' + r_id);
    reasoner_button.setAttribute('aria-controls', 'r' + r_id);
    reasoner_button.textContent = 'Reasoner (' + r_id + ')';
    reasoners_tabs.appendChild(reasoner_tab);

    const reasoners_content = document.getElementById('reasoners-content');
    const reasoner_template = document.getElementById('reasoner-content-template');
    const reasoner = reasoner_template.content.cloneNode(true).querySelector('div');
    if (!reasoners_content.hasChildNodes())
        reasoner.classList.add('show', 'active');
    reasoner.setAttribute('id', 'r' + r_id);
    reasoner.setAttribute('aria-labelledby', 'r' + r_id + '-but');
    reasoners_content.appendChild(reasoner);

    const timelines_div = document.createElement('div');
    timelines_div.setAttribute('id', 'r' + r_id + '-timelines');
    timelines_div.classList.add('h-50');
    timelines_div.style.backgroundColor = 'white';
    reasoner.appendChild(timelines_div);

    const graph_div = document.createElement('div');
    graph_div.setAttribute('id', 'r' + r_id + '-graph');
    graph_div.classList.add('h-50');
    graph_div.style.backgroundColor = 'white';
    reasoner.appendChild(graph_div);

    reasoners.set(r_id, new Reasoner(timelines_div, graph_div));
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
            case 0: return '(' + id + ') created'
            case 1: return '(' + id + ') destroyed'
            case 2: return '(' + id + ') idle'
            case 3: return '(' + id + ') reasoning'
            case 4: return '(' + id + ') executing'
            case 5: return '(' + id + ') finished'
            case 6: return '(' + id + ') inconsistent'
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
        case 4: return 'recognized'
        case 5: return 'waiting'
        default: return '-';
    }
}