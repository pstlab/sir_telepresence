const config = {
    'host': 'localhost',
    'service_port': 7000,
    'websocket_port': 8884
};
const online_icon = 'bi-check-circle';
const offline_icon = 'bi-circle';

let user;
let ws;
const users = new Map();

let current_house;

$(window).on('load', function () {
    const email = localStorage.getItem('email');
    const password = localStorage.getItem('password');
    if (email && password) {
        const form = new FormData();
        form.append('email', email);
        form.append('password', password);
        fetch('http://' + config.host + ':' + config.service_port + '/login', {
            method: 'post',
            body: form
        }).then(response => {
            if (response.ok) {
                response.json().then(data => { setUser(data); });
            } else {
                localStorage.removeItem('email');
                localStorage.removeItem('password');
                $.get('body_guest.html', function (data) { $('#sirobotics-body').append(data); });
            }
        });
    } else {
        $.get('body_guest.html', function (data) { $('#sirobotics-body').append(data); });
    }
});

function setUser(usr) {
    user = usr;
    $('#body-guest').remove();
    if (user.roles.includes('Admin')) {
        $.get('body_admin.html', function (data) {
            $('#sirobotics-body').append(data);
            $('#account-menu').text(user.firstName);
            $('#profile-email').val(user.email);
            $('#profile-first-name').val(user.firstName);
            $('#profile-last-name').val(user.lastName);

            // we set the users..
            fetch('http://' + config.host + ':' + config.service_port + '/users', {
                method: 'get',
                headers: { 'Authorization': 'Basic ' + user.id }
            }).then(response => {
                if (response.ok) {
                    response.json().then(data => {
                        const users_list = $('#users-list');
                        const user_row_template = $('#user-row');
                        const new_house_users_list = $('#new-house-users-list');
                        const new_house_user_row_template = $('#new-house-user-row');
                        for (const c_user of data.sort((a, b) => (a.lastName + a.firstName).localeCompare(b.lastName + b.firstName))) {
                            users.set(c_user.id, c_user);
                            if (c_user.id != user.id)
                                create_user_row(users_list, user_row_template, c_user.id, c_user);
                            create_new_house_user_row(new_house_users_list, new_house_user_row_template, c_user.id, c_user);
                        }
                    });
                } else
                    alert(response.statusText);
            });

            // we set the houses..
            fetch('http://' + config.host + ':' + config.service_port + '/houses', {
                method: 'get',
                headers: { 'Authorization': 'Basic ' + user.id }
            }).then(response => {
                if (response.ok) {
                    response.json().then(data => {
                        const houses_list = $('#houses-list');
                        const house_row_template = $('#house-row');
                        for (const c_house of data.sort((a, b) => a.name.localeCompare(b.name)))
                            create_house_row(houses_list, house_row_template, c_house.id, c_house);
                    });
                } else
                    alert(response.statusText);
            });

            // we set the device types..
            fetch('http://' + config.host + ':' + config.service_port + '/device_types', {
                method: 'get',
                headers: { 'Authorization': 'Basic ' + user.id }
            }).then(response => {
                if (response.ok) {
                    response.json().then(data => {
                        const new_house_sensor_types_list = $('#new-house-sensor-type');
                        const new_house_robot_types_list = $('#new-house-robot-type');
                        const sensor_types_list = $('#sensor-types-list');
                        const sensor_type_row_template = $('#sensor-type-row');
                        const robot_types_list = $('#robot-types-list');
                        const robot_type_row_template = $('#robot-type-row');
                        for (const c_device_type of data.sort((a, b) => a.name.localeCompare(b.name)))
                            switch (c_device_type.type) {
                                case 'sensor':
                                    new_house_sensor_types_list.append($('<option>', { value: c_device_type.id, text: c_device_type.name }));
                                    create_sensor_type_row(sensor_types_list, sensor_type_row_template, c_device_type.id, c_device_type);
                                    break;
                                case 'robot':
                                    new_house_robot_types_list.append($('<option>', { value: c_device_type.id, text: c_device_type.name }));
                                    create_robot_type_row(robot_types_list, robot_type_row_template, c_device_type.id, c_device_type);
                                    break;
                                default:
                                    console.error('invalid device type ' + c_device_type.type);
                                    break;
                            }
                    });
                } else
                    alert(response.statusText);
            });
        });
    }
    else if (user.roles.includes('User')) {
        $.get('body_user.html', function (data) {
            $('#sirobotics-body').append(data);
            $('#account-menu').text(user.firstName);
            $('#profile-email').val(user.email);
            $('#profile-first-name').val(user.firstName);
            $('#profile-last-name').val(user.lastName);
        });
    }
}

function login() {
    const email = $('#login-email').val();
    const password = $('#login-password').val();
    const form = new FormData();
    form.append('email', email);
    form.append('password', password);
    fetch('http://' + config.host + ':' + config.service_port + '/login', {
        method: 'post',
        body: form
    }).then(response => {
        if (response.ok) {
            localStorage.setItem('email', email);
            localStorage.setItem('password', password);
            location.reload();
        } else
            alert(response.statusText);
    });
}

function logout() {
    localStorage.removeItem('email');
    localStorage.removeItem('password');
    location.reload();
}

function signin() {
    const email = $('#signin-email').val();
    const password = $('#signin-password').val();
    const first_name = $('#signin-first-name').val();
    const last_name = $('#signin-last-name').val();
    const form = new FormData();
    form.append('email', email);
    form.append('password', password);
    form.append('first_name', first_name);
    form.append('last_name', last_name);
    fetch('http://' + config.host + ':' + config.service_port + '/user', {
        method: 'post',
        body: form
    }).then(response => {
        if (response.ok) {
            localStorage.setItem('email', email);
            localStorage.setItem('password', password);
            location.reload();
        } else
            alert(response.statusText);
    });
}

function delete_user() {
    fetch('http://' + config.host + ':' + config.service_port + '/user/' + user.id, {
        method: 'delete',
        headers: { 'Authorization': 'Basic ' + user.id }
    }).then(response => {
        if (response.ok) {
            logout();
        } else
            alert(response.statusText);
    });
}

function update_user() {
    user.firstName = $('#profile-first-name').val();
    user.lastName = $('#profile-last-name').val();
    fetch('http://' + config.host + ':' + config.service_port + '/user/' + user.id, {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: JSON.stringify(user)
    }).then(response => {
        if (response.ok) {
            $('#account-menu').text(user.firstName);
            $('#profile-modal').modal('hide');
        } else
            alert(response.statusText);
    });
}

function new_house() {
    const form = new FormData();
    form.append('name', $('#new-house-name').val());
    form.append('description', $('#new-house-description').val());
    fetch('http://' + config.host + ':' + config.service_port + '/house', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-house-name').val('');
            $('#new-house-description').val('');
            response.json().then(house => {
                create_house_row($('#houses-list'), $('#house-row'), house.id, house);
            });
        } else
            alert(response.statusText);
    });
}

function new_house_sensor() {
    const form = new FormData();
    form.append('house_id', current_house.id);
    form.append('name', $('#new-house-sensor-name').val());
    form.append('type_id', $('#new-house-sensor-type').val());
    fetch('http://' + config.host + ':' + config.service_port + '/device', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-house-sensor-name').val('');
            $('#new-house-sensor-type').val('');
            response.json().then(sensor => {
                $('#new-house-sensor-type').append($('<option>', { value: sensor.id, text: sensor.name }));
                create_house_sensor_row($('#house-sensors-list'), $('#house-sensor-row'), sensor.id, sensor);
            });
        } else
            alert(response.statusText);
    });
}

function new_house_user() {
    $('#new-house-users-list').find('input:checked').each(function () {
        const user_id = this.getAttribute('user_id');
        fetch('http://' + config.host + ':' + config.service_port + '/assign/?house_id=' + current_house.id + '&user_id=' + user_id, {
            method: 'post',
            headers: { 'Authorization': 'Basic ' + user.id }
        }).then(response => {
            if (response.ok) {
                create_house_user_row($('#house-users-list'), $('#house-user-row'), user_id, users.get(user_id));
            } else
                alert(response.statusText);
        });
    });
}

function new_house_robot() {
    const form = new FormData();
    form.append('house_id', current_house.id);
    form.append('name', $('#new-house-robot-name').val());
    form.append('type_id', $('#new-house-robot-type').val());
    fetch('http://' + config.host + ':' + config.service_port + '/device', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-house-robot-name').val('');
            $('#new-house-robot-type').val('');
            response.json().then(robot => {
                $('#new-house-robot-type').append($('<option>', { value: robot.id, text: robot.name }));
                create_house_robot_row($('#house-robots-list'), $('#house-robot-row'), robot.id, robot);
            });
        } else
            alert(response.statusText);
    });
}

function new_sensor_type() {
    const form = new FormData();
    form.append('name', $('#new-sensor-type-name').val());
    form.append('description', $('#new-sensor-type-description').val());
    form.append('category', 0);
    fetch('http://' + config.host + ':' + config.service_port + '/sensor_type', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-sensor-type-name').val('');
            $('#new-sensor-type-description').val('');
            $('#new-sensor-type-category').val('');
            response.json().then(sensor_type => {
                sensor_types.set(sensor_type.id, sensor_type);
                create_sensor_type_row($('#sensor-types-list'), $('#sensor-type-row'), sensor_type.id, sensor_type);
            });
        } else
            alert(response.statusText);
    });
}

function new_robot_type() {
    const form = new FormData();
    form.append('name', $('#new-robot-type-name').val());
    form.append('description', $('#new-robot-type-description').val());
    form.append('category', $('#new-robot-type-category').val());
    fetch('http://' + config.host + ':' + config.service_port + '/robot_type', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-robot-type-name').val('');
            $('#new-robot-type-description').val('');
            $('#new-robot-type-category').val('');
            response.json().then(robot_type => {
                robot_types.set(robot_type.id, robot_type);
                create_robot_type_row($('#robot-types-list'), $('#robot-type-row'), robot_type.id, robot_type);
            });
        } else
            alert(response.statusText);
    });
}

function create_user_row(users_list, template, id, user) {
    const user_row = template[0].content.cloneNode(true);
    const row_content = user_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    var online_span = divs[0].childNodes[0];
    online_span.id += id;
    online_span.classList.add(user.online ? online_icon : offline_icon);
    divs[0].append(user.lastName + ', ' + user.firstName);
    users_list.append(user_row);
}

function create_new_house_user_row(users_list, template, id, user) {
    const user_row = template[0].content.cloneNode(true);
    const row_content = user_row.querySelector('.list-group-item');
    row_content.childNodes[0].id += id;
    row_content.childNodes[0].setAttribute('user_id', id);
    row_content.childNodes[1].htmlFor += id;
    row_content.childNodes[1].append(user.lastName + ', ' + user.firstName);
    users_list.append(user_row);
}

function create_house_row(houses_list, template, id, house) {
    const house_row = template[0].content.cloneNode(true);
    const row_content = house_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(house.id);
    divs[1].append(house.name);
    houses_list.append(house_row);

    $('#house-' + id).on('show.bs.tab', function (event) {
        current_house = house;
        $('#house-id').val(house.id);
        $('#house-name').val(house.name);
        $('#house-description').val(house.description);

        const sensors_list = $('#house-sensors-list');
        const sensor_row_template = $('#house-sensor-row');
        const robots_list = $('#house-robots-list');
        const robot_row_template = $('#house-robot-row');
        for (const c_device of house.devices.sort((a, b) => a.name.localeCompare(b.name)))
            switch (c_device.type.type) {
                case 'sensor':
                    create_house_sensor_row(sensors_list, sensor_row_template, c_device.id, c_device);
                    break;
                case 'robot':
                    create_house_robot_row(robots_list, robot_row_template, c_device.id, c_device);
                    break;
                default:
                    console.error('invalid device type ' + c_device.type);
                    break;
            }

        const users_list = $('#house-users-list');
        const user_row_template = $('#house-user-row');
        for (const c_user of house.users.sort((a, b) => (a.lastName + a.firstName).localeCompare(b.lastName + b.firstName)))
            create_house_user_row(users_list, user_row_template, c_user.id, c_user);

        $('#house-save').click(function () {
            // we set the device types..
            fetch('http://' + config.host + ':' + config.service_port + '/house_params/?house_id=' + id, {
                method: 'get',
                headers: { 'Authorization': 'Basic ' + user.id }
            }).then(response => {
                if (response.ok) {
                    response.text().then(data => {
                        download('params.yaml', data);
                    });
                } else
                    alert(response.statusText);
            });
        });
    });
}

function create_sensor_type_row(sensor_types_list, template, id, sensor_type) {
    const sensor_type_row = template[0].content.cloneNode(true);
    const row_content = sensor_type_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(sensor_type.name);
    sensor_types_list.append(sensor_type_row);
}

function create_robot_type_row(robot_types_list, template, id, robot_type) {
    const robot_type_row = template[0].content.cloneNode(true);
    const row_content = robot_type_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(robot_type.name);
    robot_types_list.append(robot_type_row);
}

function create_house_sensor_row(sensors_list, template, id, sensor) {
    const sensor_row = template[0].content.cloneNode(true);
    const row_content = sensor_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(sensor.name);
    sensors_list.append(sensor_row);
}

function create_house_user_row(users_list, template, id, user) {
    const user_row = template[0].content.cloneNode(true);
    const row_content = user_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    var online_span = divs[0].childNodes[0];
    online_span.id += id;
    online_span.classList.add(user.online ? online_icon : offline_icon);
    divs[0].append(user.lastName + ', ' + user.firstName);
    users_list.append(user_row);
}

function create_house_robot_row(robots_list, template, id, robot) {
    const robot_row = template[0].content.cloneNode(true);
    const row_content = robot_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(robot.name);
    robots_list.append(robot_row);
}

function download(filename, text) {
    var element = document.createElement('a');
    element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
    element.setAttribute('download', filename);

    element.style.display = 'none';
    document.body.appendChild(element);

    element.click();

    document.body.removeChild(element);
}