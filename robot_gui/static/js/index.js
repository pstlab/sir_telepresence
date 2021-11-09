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
    console.log('Service request:' + request.text);
    document.getElementById('robot_face').src = 'static/faces/' + request.text + '.png';
    response['success'] = true;
    return true;
});

var listen = new ROSLIB.Service({ ros: ros, name: '/listen', serviceType: 'std_srvs/Empty' });

var system_state_listener = new ROSLIB.Topic({ ros: ros, name: '/system_state', messageType: 'msgs/system_state' });
system_state_listener.subscribe(function (message) {
    console.log('Received message on ' + system_state_listener.name + ': ' + message.data);
});

function start_listen() {
    listen.callService(new ROSLIB.ServiceRequest({}), function (result) { });
}