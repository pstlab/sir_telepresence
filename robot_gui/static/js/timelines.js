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

var set_face_service = new ROSLIB.Service({ ros: ros, name: '/set_face', serviceType: 'msgs/set_string' });
set_face_service.advertise(function (request, response) {
    console.log('Setting robot face:' + request.text);
    response['success'] = true;
    return true;
});

var reasoner_created_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_created', messageType: 'std_msgs/UInt64' });
reasoner_created_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was created..');
});

var reasoner_destroyed_listener = new ROSLIB.Topic({ ros: ros, name: '/reasoner_destroyed', messageType: 'std_msgs/UInt64' });
reasoner_destroyed_listener.subscribe(function (message) {
    console.log('Reasoner ' + message.data + ' was destroyed..');
});

var flaw_created_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_created', messageType: 'deliberative_tier/flaw_created' });
flaw_created_listener.subscribe(function (message) {
});

var flaw_state_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_state_changed', messageType: 'deliberative_tier/flaw_state_changed' });
flaw_state_changed_listener.subscribe(function (message) {
});

var flaw_cost_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_cost_changed', messageType: 'deliberative_tier/flaw_cost_changed' });
flaw_cost_changed_listener.subscribe(function (message) {
});

var flaw_position_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/flaw_position_changed', messageType: 'deliberative_tier/flaw_position_changed' });
flaw_position_changed_listener.subscribe(function (message) {
});

var current_flaw_listener = new ROSLIB.Topic({ ros: ros, name: '/current_flaw', messageType: 'deliberative_tier/current_flaw' });
current_flaw_listener.subscribe(function (message) {
});

var resolver_created_listener = new ROSLIB.Topic({ ros: ros, name: '/resolver_created', messageType: 'deliberative_tier/resolver_created' });
resolver_created_listener.subscribe(function (message) {
});

var resolver_state_changed_listener = new ROSLIB.Topic({ ros: ros, name: '/resolver_state_changed', messageType: 'deliberative_tier/resolver_state_changed' });
resolver_state_changed_listener.subscribe(function (message) {
});

var current_resolver_listener = new ROSLIB.Topic({ ros: ros, name: '/current_resolver', messageType: 'deliberative_tier/current_resolver' });
current_resolver_listener.subscribe(function (message) {
});

var causal_link_added_listener = new ROSLIB.Topic({ ros: ros, name: '/causal_link_added', messageType: 'deliberative_tier/causal_link_added' });
causal_link_added_listener.subscribe(function (message) {
});