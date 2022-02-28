const ros_host = 'localhost'
const ros_port = '9090'

const state = {
    sequencer_state: null,
    deliberative_state: new Map(),
    dialogue_state: null
};

const reasoners = new Map();

const ros = new ROSLIB.Ros({ url: 'ws://' + ros_host + ':' + ros_port });

ros.on('connection', function () {
    console.log('Connected to websocket server..');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed..');
});

const get_deliberative_state = new ROSLIB.Service({ ros: ros, name: '/get_state', serviceType: 'deliberative_tier/get_state' });
get_deliberative_state.callService(ROSLIB.ServiceRequest({}), function (result) {
    for (const r of result.timelines) {
        create_reasoner(r.reasoner_id);
        r.state = JSON.parse(r.state);
        r.timelines.forEach((tl, i) => r.timelines[i] = JSON.parse(tl));
        reasoners.get(r.reasoner_id).state_changed(r);
    }
    for (const g of result.graphs) {
        for (const f of g.flaws) f.data = JSON.parse(f.data);
        for (const r of g.resolvers) r.data = JSON.parse(r.data);
        if (g.flaw_id) g.current_flaw = g.flaw_id;
        if (g.resolver_id) g.current_resolver = g.resolver_id;
        reasoners.get(g.reasoner_id).graph(g);
    }

    init_ros_services();
});

function init_ros_services() {
    const set_face_service = new ROSLIB.Service({ ros: ros, name: '/set_face', serviceType: 'dialogue_manager/face_to_show' });
    set_face_service.advertise(function (request, response) {
        console.log('Setting robot face:' + request.facial_expression);
        document.getElementById('face_div').classList.remove('d-none');
        document.getElementById('image_div').classList.add('d-none');
        document.getElementById('audio_div').classList.add('d-none');
        document.getElementById('video_div').classList.add('d-none');
        document.getElementById('html_div').classList.add('d-none');
        document.getElementById('question_div').classList.add('d-none');

        document.getElementById('robot_face').src = 'static/faces/' + request.facial_expression + '.gif';
        document.getElementById('robot_face').height = 700
        response['success'] = true;
        return true;
    });

    const show_image_service = new ROSLIB.Service({ ros: ros, name: '/show_image', serviceType: 'dialogue_manager/image_to_show' });
    show_image_service.advertise(function (request, response) {
        console.log('Showing image:' + request.src);
        document.getElementById('face_div').classList.add('d-none');
        document.getElementById('image_div').classList.remove('d-none');
        document.getElementById('audio_div').classList.add('d-none');
        document.getElementById('video_div').classList.add('d-none');
        document.getElementById('html_div').classList.add('d-none');
        document.getElementById('question_div').classList.add('d-none');

        document.getElementById('image').src = request.src;
        document.getElementById('image').alt = request.alt;
        response['success'] = true;
        return true;
    });

    const play_audio_service = new ROSLIB.Service({ ros: ros, name: '/play_audio', serviceType: 'dialogue_manager/audio_to_play' });
    play_audio_service.advertise(function (request, response) {
        console.log('Playing audio:' + request.src);
        document.getElementById('face_div').classList.add('d-none');
        document.getElementById('image_div').classList.add('d-none');
        document.getElementById('audio_div').classList.remove('d-none');
        document.getElementById('video_div').classList.add('d-none');
        document.getElementById('html_div').classList.add('d-none');
        document.getElementById('question_div').classList.add('d-none');

        const audio = document.getElementById('audio');
        let sources = [];
        request.videos.forEach(src => {
            let source = document.createElement('source');
            source.src = src.src;
            source.type = src.type;
            sources.push(source);
        });
        audio.replaceChildren(sources);

        audio.play();
        response['success'] = true;
        return true;
    });

    const play_video_service = new ROSLIB.Service({ ros: ros, name: '/play_video', serviceType: 'dialogue_manager/video_to_play' });
    play_video_service.advertise(function (request, response) {
        console.log('Playing video:' + request.src);
        document.getElementById('face_div').classList.add('d-none');
        document.getElementById('image_div').classList.add('d-none');
        document.getElementById('audio_div').classList.add('d-none');
        document.getElementById('video_div').classList.remove('d-none');
        document.getElementById('html_div').classList.add('d-none');
        document.getElementById('question_div').classList.add('d-none');

        const video = document.getElementById('video');
        let sources = [];
        request.videos.forEach(src => {
            let source = document.createElement('source');
            source.src = src.src;
            source.type = src.type;
            sources.push(source);
        });
        video.replaceChildren(sources);

        video.play();
        response['success'] = true;
        return true;
    });

    const show_page_service = new ROSLIB.Service({ ros: ros, name: '/show_page', serviceType: 'dialogue_manager/page_to_show' });
    show_page_service.advertise(function (request, response) {
        console.log('Showing page:' + request.src);
        document.getElementById('face_div').classList.add('d-none');
        document.getElementById('image_div').classList.add('d-none');
        document.getElementById('audio_div').classList.add('d-none');
        document.getElementById('video_div').classList.add('d-none');
        document.getElementById('html_div').classList.remove('d-none');
        document.getElementById('question_div').classList.add('d-none');

        document.getElementById('html').src = request.src;
        document.getElementById('html').title = request.title;
        response['success'] = true;
        return true;
    });

    const ask_question_service = new ROSLIB.Service({ ros: ros, name: '/ask_question', serviceType: 'dialogue_manager/question_to_ask' });
    ask_question_service.advertise(function (request, response) {
        console.log('Asking question:' + request.text);
        document.getElementById('face_div').classList.remove('d-none');
        document.getElementById('image_div').classList.add('d-none');
        document.getElementById('audio_div').classList.add('d-none');
        document.getElementById('video_div').classList.add('d-none');
        document.getElementById('html_div').classList.add('d-none');
        document.getElementById('question_div').classList.remove('d-none');

        document.getElementById('robot_face').src = 'static/faces/' + request.facial_expression + '.gif';
        document.getElementById('robot_face').height = 300
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

    const toast_service = new ROSLIB.Service({ ros: ros, name: '/toast', serviceType: 'dialogue_manager/toast_to_show' });
    toast_service.advertise(function (request, response) {
        console.log('Showing toast:' + request.text);
        const toast_div = document.getElementById('recognized_speech');
        const toast_text = document.getElementById('recognized_speech_text');
        toast_text.innerHTML = request.text;
        var toast = new bootstrap.Toast(toast_div)
        toast.show()
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

    const listen_service = new ROSLIB.Service({ ros: ros, name: '/listen', serviceType: 'std_srvs/Trigger' });
    document.getElementById('listen-button').onclick = (event) => start_listen(listen_service);

    const sequencer_state_listener = new ROSLIB.Topic({ ros: ros, name: '/sequencer_state', messageType: 'sequencer_tier/sequencer_state' });
    sequencer_state_listener.subscribe(function (message) {
        state.sequencer_state = message.sequencer_state;
        print_state();
    });

    const deliberative_state_listener = new ROSLIB.Topic({ ros: ros, name: '/deliberative_state', messageType: 'deliberative_tier/deliberative_state' });
    deliberative_state_listener.subscribe(function (message) {
        state.deliberative_state.set(message.reasoner_id, message.deliberative_state);

        if (message.deliberative_state == 3 || message.deliberative_state == 4) {
            const reasoner = reasoners.get(message.reasoner_id);

            reasoner.current_flaw = undefined;
            reasoner.current_resolver = undefined;
            reasoner.network.unselectAll();
        }
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
        create_reasoner(message.data);
    });

    const reasoner_destroyed_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_destroyed', messageType: 'std_msgs/UInt64' });
    reasoner_destroyed_listener.subscribe(function (message) {
        console.log('Reasoner ' + message.data + ' was destroyed..');

        document.getElementById('r' + message.data + '-tab').remove();
        document.getElementById('r' + message.data).remove();

        reasoners.delete(message.data);
    });

    const graph_listener = new ROSLIB.Topic({ ros: ros, name: '/graph', messageType: 'deliberative_tier/graph' });
    graph_listener.subscribe(function (message) {
        switch (message.update) {
            case 0: // graph_changed
                for (const f of message.flaws) f.data = JSON.parse(f.data);
                for (const r of message.resolvers) r.data = JSON.parse(r.data);
                if (message.flaw_id) message.current_flaw = message.flaw_id;
                if (message.resolver_id) message.current_resolver = message.resolver_id;
                reasoners.get(message.reasoner_id).graph(message);
                break;
            case 1: // flaw_created
                message.flaws[0].data = JSON.parse(message.flaws[0].data);
                reasoners.get(message.reasoner_id).flaw_created(message.flaws[0]);
                break;
            case 2: // flaw_state_changed
                reasoners.get(message.reasoner_id).flaw_state_changed(message.flaws[0]);
                break;
            case 3: // flaw_cost_changed
                reasoners.get(message.reasoner_id).flaw_cost_changed(message.flaws[0]);
                break;
            case 4: // flaw_position_changed
                reasoners.get(message.reasoner_id).flaw_position_changed(message.flaws[0]);
                break;
            case 5: // current_flaw
                message.id = message.flaw_id;
                reasoners.get(message.reasoner_id).current_flaw_changed(message);
                break;
            case 6: // resolver_created
                message.resolvers[0].data = JSON.parse(message.resolvers[0].data);
                reasoners.get(message.reasoner_id).resolver_created(message.resolvers[0]);
                break;
            case 7: // resolver_state_changed
                reasoners.get(message.reasoner_id).resolver_state_changed(message.resolvers[0]);
                break;
            case 8: // current_resolver
                message.id = message.resolver_id;
                reasoners.get(message.reasoner_id).current_resolver_changed(message);
                break;
            case 9: // causal_link_added
                reasoners.get(message.reasoner_id).causal_link_added(message);
                break;
            default:
                break;
        }
    });

    const timelines_listener = new ROSLIB.Topic({ ros: ros, name: '/timelines', messageType: 'deliberative_tier/timelines' });
    timelines_listener.subscribe(function (message) {
        console.log(message);
        switch (message.update) {
            case 0: // state_changed
                message.state = JSON.parse(message.state);
                message.timelines.forEach((tl, i) => message.timelines[i] = JSON.parse(tl));
                reasoners.get(message.reasoner_id).state_changed(message);
                break;
            case 1: // time_changed
                reasoners.get(message.reasoner_id).tick(message);
                break;
            case 2: // executing_changed
                reasoners.get(message.reasoner_id).executing_changed(message);
                break;
            default:
                break;
        }
    });
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
    reasoner_button.append('Reasoner (' + r_id + ')');
    reasoners_tabs.append(reasoner_tab);

    const reasoners_content = document.getElementById('reasoners-content');
    const reasoner_template = document.getElementById('reasoner-content-template');
    const reasoner = reasoner_template.content.cloneNode(true).querySelector('div');
    if (!reasoners_content.hasChildNodes())
        reasoner.classList.add('show', 'active');
    reasoner.setAttribute('id', 'r' + r_id);
    reasoner.setAttribute('aria-labelledby', 'r' + r_id + '-but');
    reasoners_content.append(reasoner);

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

function sv_value_class(val) {
    switch (val.atoms.length) {
        case 0: return 'sv-empty';
        case 1: return 'sv-consistent';
        default: return 'sv-inconsistent';
    }
}

function rr_value_class(rr, val) { return rr.capacity.num / rr.capacity.den < val.usage.num / val.usage.den ? 'rr-inconsistent' : 'rr-consistent' }

function atom_content(atm) { return atm.predicate + '(' + atm.pars.filter(par => par.name != 'start' && par.name != 'end' && par.name != 'duration' && par.name != 'tau').map(par => par.name).sort().join(', ') + ')'; }

function atom_title(atm) { return '\u03C3' + atm.sigma + ' ' + atm.predicate + '(' + atm.pars.filter(par => par.name != 'start' && par.name != 'end' && par.name != 'duration' && par.name != 'tau').map(par => par_to_string(par)).sort().join(', ') + ')'; }

function par_to_string(par) {
    switch (par.type) {
        case 'bool': return par.name + ': ' + par.value;
        case 'real': return par.name + ': ' + par.value.num / par.value.den;
        default: return par.name;
    }
}

function color(n) {
    switch (n.state) {
        case 0: // False
            return chroma('lightgray').hex();
        case 1: // True
        case 2: // Undefined
            if (n.cost < Number.POSITIVE_INFINITY)
                return color_domain(Math.min(max_cost, n.cost)).hex();
            else
                return chroma('darkgray').hex();
        default:
            break;
    }
}

function stroke_dasharray(n) {
    switch (n.state) {
        case 0: // False
            return [2, 2];
        case 1: // True
            return false;
        case 2: // Undefined
            return [4, 4];
        default:
            break;
    }
}

function flaw_label(flaw) {
    switch (flaw.data.type) {
        case 'fact':
            return 'fact \u03C3' + flaw.data.sigma + ' ' + flaw.data.predicate;
        case 'goal':
            return 'goal \u03C3' + flaw.data.sigma + ' ' + flaw.data.predicate;
        case 'enum':
            return 'enum';
        case 'bool':
            return 'bool';
        default:
            switch (flaw.data.phi) {
                case 'b0':
                case '\u00ACb0':
                    return flaw.data.type;
                default:
                    return flaw.data.phi.replace('b', '\u03C6') + ' ' + flaw.data.type;
            }
    }
}

function flaw_tooltip(flaw) {
    switch (flaw.data.phi) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + flaw.cost + ', pos: ' + flaw.pos.lb;
        default:
            return flaw.data.phi.replace('b', '\u03C6') + ', cost: ' + flaw.cost + ', pos: ' + flaw.pos.lb;
    }
}

function resolver_label(resolver) {
    if (resolver.data.type)
        switch (resolver.data.type) {
            case 'activate':
                return 'activate';
            case 'unify':
                return 'unify';
            case 'assignment':
                return resolver.data.val;
            default:
                switch (resolver.data.rho) {
                    case 'b0':
                    case '\u00ACb0':
                        return resolver.data.type;
                    default:
                        return resolver.data.rho.replace('b', '\u03C1') + ' ' + resolver.data.type;
                }
        }
    switch (resolver.data.rho) {
        case 'b0':
            return '\u22A4';
        case '\u00ACb0':
            return '\u22A5';
        default:
            return resolver.data.rho.replace('b', '\u03C1');
    }
}

function resolver_tooltip(resolver) {
    switch (resolver.data.rho) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + resolver.cost;
        default:
            return resolver.data.rho.replace('b', '\u03C1') + ', cost: ' + resolver.cost;
    }
}

function start_listen(listen_service) {
    listen_service.callService(new ROSLIB.ServiceRequest({}), function (result) {
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
        default: return '-';
    }
}