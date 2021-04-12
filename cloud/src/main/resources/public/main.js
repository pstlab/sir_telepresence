const config = {
    'host': '192.168.1.101',
    'service_port': 7000,
    'websocket_port': 8884
};
const online_icon = 'bi-check-circle';
const offline_icon = 'bi-circle';
let user;
let ws;

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
                        for (const c_user of data.sort((a, b) => (a.lastName + a.firstName).localeCompare(b.lastName + b.firstName)))
                            if (c_user.id != user.id)
                                create_user_row(users_list, user_row_template, c_user.id, c_user);
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
                        const sensor_types_list = $('#sensor-types-list');
                        const sensor_type_row_template = $('#sensor-type-row');
                        const robot_types_list = $('#robot-types-list');
                        const robot_type_row_template = $('#robot-type-row');
                        for (const c_device_type of data.sort((a, b) => a.name.localeCompare(b.name)))
                            switch (c_device_type.type) {
                                case 'sensor':
                                    create_sensor_type_row(sensor_types_list, sensor_type_row_template, c_device_type.id, c_device_type);
                                    break;
                                case 'robot':
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

function create_house_row(houses_list, template, id, house) {
    const house_row = template[0].content.cloneNode(true);
    const row_content = house_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(house.id);
    divs[1].append(house.name);
    houses_list.append(house_row);

    $('#house-' + id).on('show.bs.tab', function (event) {
        $('#house-id').val(house.id);
        $('#house-name').val(house.name);
        $('#house-description').val(house.description);
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