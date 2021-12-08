document.getElementById("home-nav-item").classList.remove("active");
document.getElementById("timelines-nav-item").classList.add("active");

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

var set_face_service = new ROSLIB.Service({ ros: ros, name: '/set_face', serviceType: 'dialogue_manager/set_string' });
set_face_service.advertise(function (request, response) {
    console.log('Setting robot face:' + request.text);
    response['success'] = true;
    return true;
});

var reasoner_created_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_created', messageType: 'std_msgs/UInt64' });
reasoner_created_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was created..');

    const reasoners_tabs = document.getElementById('reasoners-tabs');
    const reasoner_tab_template = document.getElementById('reasoner-tab-template');
    const reasoner_tab = reasoner_tab_template.content.cloneNode(true).querySelector('li');
    const reasoner_button = reasoner_tab.querySelector('button');
    if (!reasoners_tabs.hasChildNodes())
        reasoner_button.classList.add('active');
    reasoner_button.setAttribute('id', 'r' + message.data + '-tab');
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
    reasoner.setAttribute('aria-labelledby', 'r' + message.data + '-tab');
    reasoners_content.append(reasoner);
});

var reasoner_destroyed_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_destroyed', messageType: 'std_msgs/UInt64' });
reasoner_destroyed_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was destroyed..');
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