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
                        const device_types_list = $('#device-types-list');
                        const device_type_row_template = $('#device-type-row');
                        for (const c_device_type of data.sort((a, b) => a.name.localeCompare(b.name)))
                            create_device_type_row(device_types_list, device_type_row_template, c_device_type.id, c_device_type);
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

function new_device_type() {
    const form = new FormData();
    form.append('name', $('#new-device-type-name').val());
    form.append('description', $('#new-device-type-description').val());
    form.append('category', $('#new-device-type-category').val());
    fetch('http://' + config.host + ':' + config.service_port + '/device_type', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-device-type-name').val('');
            $('#new-device-type-description').val('');
            $('#new-device-type-category').val('');
            response.json().then(device_type => {
                create_device_type_row($('#device-types-list'), $('#device-type-row'), device_type.id, device_type);
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
}

function create_device_type_row(device_types_list, template, id, device_type) {
    const device_type_row = template[0].content.cloneNode(true);
    const row_content = device_type_row.querySelector('.list-group-item');
    row_content.id += id;
    const divs = row_content.querySelectorAll('div');
    divs[0].append(device_type.name);
    device_types_list.append(device_type_row);
}