const ros_host = 'localhost'
const ros_port = '9090'
const animation = false;

const sc = chroma.scale(['#90EE90', 'yellow', '#A91101']);
let max_cost = 1;
let color_domain = sc.domain([0, max_cost]);

const state = {
    sequencer_state: null,
    deliberative_state: new Map(),
    dialogue_state: null
};

class Reasoner {
    constructor(id) {
        this.id = id;

        this.items = new Map();
        this.atoms = new Map();

        this.timelines = new vis.DataSet([]);
        this.timeline_values = new vis.DataSet([]);

        this.timeline = new vis.Timeline(document.getElementById('r' + id + '-timelines'), this.timeline_values, this.timelines, { selectable: false, showCurrentTime: false });

        this.current_time = 0;
        this.timeline.addCustomTime(this.current_time);
        this.timeline.customTimes[this.timeline.customTimes.length - 1].hammer.off("panstart panmove panend");

        this.executing_tasks = new Set();

        this.nodes = new vis.DataSet([]);
        this.edges = new vis.DataSet([]);
        this.current_flaw;
        this.current_resolver;

        this.network = new vis.Network(document.getElementById('r' + id + '-graph'), { nodes: this.nodes, edges: this.edges }, { layout: { hierarchical: { direction: "RL", } } });
    }

    state_changed(message) {
        this.items.clear(); if (message.state.items) for (const itm of message.state.items) this.items.set(parseInt(itm.id), itm);
        this.atoms.clear(); if (message.state.atoms) for (const atm of message.state.atoms) this.atoms.set(parseInt(atm.id), atm);
        this.executing_tasks.clear();
        this.timelines.update(message.timelines.map(tl => { return { id: tl.id, content: tl.name } }));
        const origin_var = message.state.exprs.find(xpr => xpr.name == 'origin');
        const horizon_var = message.state.exprs.find(xpr => xpr.name == 'horizon');
        const origin_val = origin_var.value.val.num / origin_var.value.val.den;
        const horizon_val = horizon_var.value.val.num / horizon_var.value.val.den;
        this.timeline.setWindow(origin_val - 10, origin_val == horizon_val ? horizon_val + 100 : horizon_val + 10);
        const vals = [];
        for (const tl of message.timelines)
            switch (tl.type) {
                case 'StateVariable': {
                    const sv_atms = new Set();
                    tl.values.forEach((val, id) => {
                        vals.push({
                            id: '' + tl.id + id,
                            className: sv_value_class(val),
                            start: val.from.num / val.from.den,
                            end: val.to.num / val.to.den,
                            type: 'background',
                            group: tl.id
                        });
                        for (const atm_id of val.atoms)
                            sv_atms.add(atm_id);
                    });
                    for (const atm_id of sv_atms) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        const end_var = atm.pars.find(xpr => xpr.name == 'end');
                        vals.push({
                            id: atm.id,
                            content: atom_content(atm),
                            title: atom_title(atm),
                            start: start_var.value.val.num / start_var.value.val.den,
                            end: end_var.value.val.num / end_var.value.val.den,
                            group: tl.id
                        });
                    }
                    break;
                }
                case 'Agent':
                    for (const atm_id of tl.values) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        if (start_var) {
                            const end_var = atm.pars.find(xpr => xpr.name == 'end');
                            vals.push({
                                id: atm.id,
                                content: atom_content(atm),
                                title: atom_title(atm),
                                start: start_var.value.val.num / start_var.value.val.den,
                                end: end_var.value.val.num / end_var.value.val.den,
                                group: tl.id
                            });
                        } else {
                            const at_var = atm.pars.find(xpr => xpr.name == 'at');
                            vals.push({
                                id: atm.id,
                                content: atom_content(atm),
                                start: at_var.value.val.num / at_var.value.val.den,
                                group: tl.id
                            });
                        }
                    }
                    break;
                case 'ReusableResource':
                    const sv_atms = new Set();
                    tl.values.forEach((val, id) => {
                        vals.push({
                            id: '' + tl.id + id,
                            content: val.usage.num / val.usage.den,
                            className: rr_value_class(tl, val),
                            start: val.from.num / val.from.den,
                            end: val.to.num / val.to.den,
                            type: 'background',
                            group: tl.id
                        });
                        for (const atm_id of val.atoms)
                            sv_atms.add(atm_id);
                    });
                    for (const atm_id of sv_atms) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        const end_var = atm.pars.find(xpr => xpr.name == 'end');
                        vals.push({
                            id: atm.id,
                            content: atom_content(atm),
                            title: atom_title(atm),
                            start: start_var.value.val.num / start_var.value.val.den,
                            end: end_var.value.val.num / end_var.value.val.den,
                            group: tl.id
                        });
                    }
                    break;
                default:
                    break;
            }
        this.timeline_values.update(vals);
        for (const t of message.executing)
            this.executing_tasks.add(t);
        this.timeline.setSelection(Array.from(this.executing_tasks), { focus: true, animate: animation });
        this.current_time = message.time.num / message.time.den;
        this.timeline.setCustomTime(this.current_time);
    }

    time_changed(message) {
        this.current_time = message.time.num / message.time.den;
        this.timeline.setCustomTime(this.current_time);
    }

    graph_changed(message) {
        for (const f of message.flaws) {
            const flaw = {
                type: 'flaw',
                id: f.id,
                causes: f.causes,
                state: f.state,
                cost: f.cost.num / f.cost.den,
                pos: f.pos,
                data: f.data
            };
            flaw.label = flaw_label(flaw);
            flaw.title = flaw_tooltip(flaw);
            flaw.shapeProperties = { borderDashes: stroke_dasharray(flaw) };
            if (flaw.cost != Number.POSITIVE_INFINITY && max_cost < flaw.cost)
                max_cost = flaw.cost;
            this.nodes.add(flaw);
        }
        color_domain = sc.domain([0, max_cost]);
        const all_nodes = this.nodes.get();
        all_nodes.forEach(n => n.color = color(n));
        this.nodes.update(all_nodes);

        for (const r of message.resolvers) {
            const resolver = {
                type: 'resolver',
                id: r.id,
                preconditions: r.preconditions,
                effect: r.effect,
                state: r.state,
                intrinsic_cost: r.intrinsic_cost.num / r.intrinsic_cost.den,
                data: r.data
            };
            resolver.cost = this.estimate_cost(resolver);
            resolver.label = resolver_label(resolver);
            resolver.title = resolver_tooltip(resolver);
            resolver.shape = 'box';
            resolver.shapeProperties = { borderDashes: stroke_dasharray(resolver) };
            resolver.color = color(resolver);
            this.nodes.add(resolver);
            this.edges.add({ from: r.id, to: r.effect, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
            for (const f of resolver.preconditions)
                this.edges.add({ from: f, to: resolver.id, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
        }

        if (message.current_flaw) {
            this.current_flaw = message.current_flaw;
            if (message.current_resolver) {
                this.current_resolver = message.current_resolver;
                this.network.selectNodes([this.current_flaw, this.current_resolver]);
                this.network.focus(this.current_resolver, { animation: animation });
            } else {
                this.current_resolver = undefined;
                this.network.selectNodes([this.current_flaw]);
                this.network.focus(this.current_flaw, { animation: animation });
            }
        }
    }

    flaw_created(message) {
        const flaw = {
            type: 'flaw',
            id: message.id,
            causes: message.causes,
            state: message.state,
            cost: message.cost.num / message.cost.den,
            pos: message.pos,
            data: message.data
        };
        flaw.label = flaw_label(flaw);
        flaw.title = flaw_tooltip(flaw);
        flaw.shapeProperties = { borderDashes: stroke_dasharray(message) };
        flaw.color = color(flaw);
        this.nodes.add(flaw);
        const causes = this.nodes.get(flaw.causes);
        const causes_edges = [];
        for (const c of causes) {
            c.preconditions.push(flaw.id);
            const c_res_cost = this.estimate_cost(c);
            if (c.cost != c_res_cost) {
                c.cost = c_res_cost;
                c.title = resolver_tooltip(c);
                c.color = color(c);
            }
            causes_edges.push({ from: message.id, to: c.id, arrows: { to: true }, dashes: stroke_dasharray(this.nodes.get(c)) });
        }
        this.edges.add(causes_edges);
        this.nodes.update(causes);
    }

    flaw_state_changed(message) {
        const flaw = this.nodes.get(message.id);
        flaw.state = message.state;
        flaw.shapeProperties.borderDashes = stroke_dasharray(message);
        flaw.color = color(flaw);
        this.nodes.update(flaw);
    }

    flaw_cost_changed(message) {
        const flaw = this.nodes.get(message.id);
        flaw.cost = message.cost.num / message.cost.den;
        flaw.title = flaw_tooltip(flaw);
        if (flaw.cost != Number.POSITIVE_INFINITY && max_cost < flaw.cost) {
            max_cost = flaw.cost;
            color_domain = sc.domain([0, max_cost]);
            const all_nodes = this.nodes.get();
            all_nodes.forEach(n => n.color = color(n));
            this.nodes.update(all_nodes);
        } else {
            flaw.color = color(flaw);
            this.nodes.update(flaw);
        }
        const updated_res = [];
        for (const c of flaw.causes.map(r_id => this.nodes.get(r_id))) {
            const c_res_cost = this.estimate_cost(c);
            if (c.cost != c_res_cost) {
                c.cost = c_res_cost;
                c.title = resolver_tooltip(c);
                c.color = color(c);
                updated_res.push(c);
            }
        }
        if (updated_res)
            this.nodes.update(updated_res);
    }

    flaw_position_changed(message) {
        const flaw = this.nodes.get(message.id);
        flaw.pos = message.pos;
        flaw.title = flaw_tooltip(flaw);
        this.nodes.update(flaw);
    }

    current_flaw_changed(message) {
        this.current_flaw = message.id;
        this.current_resolver = undefined;
        this.network.selectNodes([this.current_flaw]);
        this.network.focus(this.current_flaw, { animation: animation });
    }

    resolver_created(message) {
        message.cost = message.intrinsic_cost.num / message.intrinsic_cost.den;
        if (message.cost != Number.POSITIVE_INFINITY && max_cost < message.cost) {
            max_cost = message.cost;
            color_domain = sc.domain([0, max_cost]);
            const all_nodes = this.nodes.get();
            all_nodes.forEach(n => n.color = color_domain(n.cost));
            this.nodes.update(all_nodes);
        }
        const resolver = {
            type: 'resolver',
            id: message.id,
            preconditions: message.preconditions,
            effect: message.effect,
            state: message.state,
            intrinsic_cost: message.intrinsic_cost.num / message.intrinsic_cost.den,
            data: message.data
        };
        resolver.cost = this.estimate_cost(resolver);
        resolver.label = resolver_label(resolver);
        resolver.title = resolver_tooltip(resolver);
        resolver.shape = 'box';
        resolver.shapeProperties = { borderDashes: stroke_dasharray(resolver) };
        resolver.color = color(resolver);
        this.nodes.add(resolver);
        this.edges.add({ from: message.id, to: message.effect, arrows: { to: true }, dashes: stroke_dasharray(message) });
    }

    resolver_state_changed(message) {
        const resolver = this.nodes.get(message.id);
        resolver.state = message.state;
        resolver.shapeProperties.borderDashes = stroke_dasharray(resolver);
        resolver.color = color(resolver);
        this.nodes.update(resolver);
        const c_edges = this.network.getConnectedEdges(message.id);
        c_edges.forEach((e_id, i) => {
            c_edges[i] = this.edges.get(e_id);
            c_edges[i].dashes = stroke_dasharray(resolver);
        });
        this.edges.update(c_edges);
    }

    current_resolver_changed(message) {
        this.current_resolver = message.id;
        this.network.selectNodes([this.current_flaw, this.current_resolver]);
        this.network.focus(this.current_resolver, { animation: animation });
    }

    causal_link_added(message) {
        const flaw = this.nodes.get(message.flaw_id);
        const resolver = this.nodes.get(message.resolver_id);
        resolver.preconditions.push(flaw);
        this.edges.add({ from: message.flaw_id, to: message.resolver_id, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
    }

    estimate_cost(res) {
        return (res.preconditions ? Math.max.apply(Math, res.preconditions.map(f_id => this.nodes.get(f_id).cost)) : 0) + res.intrinsic_cost;
    }
}
const reasoners = new Map();

document.getElementById('listen-button').addEventListener('click', start_listen);

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

    const graph_div = document.createElement('div');
    graph_div.setAttribute('id', 'r' + message.data + '-graph');
    graph_div.style.backgroundColor = 'white';
    reasoner.appendChild(graph_div);

    reasoners.set(message.data, new Reasoner(message.data));
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
            if (message.flaw_id) message.current_flaw = message.flaw_id;
            if (message.resolver_id) message.current_resolver = message.resolver_id;
            reasoners.get(message.reasoner_id).graph_changed(message);
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
            reasoners.get(message.reasoner_id).time_changed(message);
            break;
        default:
            break;
    }
});

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

function start_listen() {
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