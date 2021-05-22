import * as config from './config.js'
import * as context from './context.js'
import { Graph, GraphData } from './modules/graph.js';
import { Timelines, TimelinesData } from './modules/timelines.js';

let current_house = undefined;
export let current_plan = undefined;

export let timelines = undefined;
export let graph = undefined;

export function init() {
    // we set the houses..
    fetch('http://' + config.host + ':' + config.service_port + '/houses', {
        method: 'get',
        headers: { 'Authorization': 'Basic ' + context.user.id }
    }).then(response => {
        if (response.ok) {
            response.json().then(data => {
                const houses_list = $('#houses-list');
                const house_row_template = $('#house-row');
                for (const house of data.sort((a, b) => a.name.localeCompare(b.name))) {
                    const house_row = create_house_row(house_row_template, house);
                    houses_list.append(house_row);
                    refine_house_row(house_row, house);
                }
            });
        } else
            alert(response.statusText);
    });

    timelines = new Timelines('timelines', window.innerWidth, window.innerHeight / 3);
    graph = new Graph('graph', window.innerWidth, window.innerHeight / 3);
}

export function new_user(user) {
    const new_house_users_list = $('#new-house-users-list');
    const new_house_user_row_template = $('#new-house-user-row');
    const new_house_user_row = create_new_house_user_row(new_house_user_row_template, user);
    new_house_users_list.append(new_house_user_row);
}

export function new_sensor_type(sensor_type) { $('#new-house-sensor-type').append($('<option>', { value: sensor_type.id, text: sensor_type.name })); }

export function new_robot_type(robot_type) { $('#new-house-robot-type').append($('<option>', { value: robot_type.id, text: robot_type.name })); }

export function new_house() {
    const form = new FormData();
    form.append('name', $('#new-house-name').val());
    form.append('description', $('#new-house-description').val());
    fetch('http://' + config.host + ':' + config.service_port + '/house', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + context.user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-house-name').val('');
            $('#new-house-description').val('');
            response.json().then(house => {
                const house_row = create_house_row($('#house-row'), house);
                $('#houses-list').append(house_row);
                refine_house_row(house_row, house);
            });
        } else
            alert(response.statusText);
    });
}

export function new_house_user() {
    $('#new-house-users-list').find('input:checked').each(function () {
        const user_id = parseInt(this.getAttribute('user_id'), 10);
        fetch('http://' + config.host + ':' + config.service_port + '/assign/?house_id=' + current_house.id + '&user_id=' + user_id, {
            method: 'post',
            headers: { 'Authorization': 'Basic ' + context.user.id }
        }).then(response => {
            if (response.ok) {
                $('#house-users-list').append(create_house_user_row($('#house-user-row'), context.users.get(user_id)));
            } else
                alert(response.statusText);
        });
    });
}

export function new_house_sensor() {
    const form = new FormData();
    form.append('house_id', current_house.id);
    form.append('name', $('#new-house-sensor-name').val());
    form.append('type_id', $('#new-house-sensor-type').val());
    fetch('http://' + config.host + ':' + config.service_port + '/device', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + context.user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-house-sensor-name').val('');
            $('#new-house-sensor-type').val('');
            response.json().then(sensor => {
                current_house.devices.push(sensor);
                $('#new-house-sensor-type').append($('<option>', { value: sensor.id, text: sensor.name }));
                $('#house-sensors-list').append(create_house_sensor_row($('#house-sensor-row'), sensor));
            });
        } else
            alert(response.statusText);
    });
}

export function new_house_robot() {
    const c_house = current_house;
    const form = new FormData();
    form.append('house_id', c_house.id);
    form.append('name', $('#new-house-robot-name').val());
    form.append('type_id', $('#new-house-robot-type').val());
    fetch('http://' + config.host + ':' + config.service_port + '/device', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + context.user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-house-robot-name').val('');
            $('#new-house-robot-type').val('');
            response.json().then(robot => {
                c_house.devices.push(robot);
                $('#new-house-robot-type').append($('<option>', { value: robot.id, text: robot.name }));
                $('#house-robots-list').append(create_house_robot_row($('#house-robot-row'), c_house.id, robot));
                refine_house_robot_row(c_house.id, robot);
                $('#plan-tabs').append(create_plan_tab($('#plan-tab'), robot));
            });
        } else
            alert(response.statusText);
    });
}

function create_house_row(template, house) {
    const house_row = template[0].content.cloneNode(true);
    const row_content = house_row.querySelector('.list-group-item');
    row_content.id += house.id;
    row_content.querySelector('.house_id').append(house.id);
    row_content.querySelector('.house_name').append(house.name);
    return house_row;
}

function refine_house_row(house_row, house) {
    $('#house-' + house.id).on('show.bs.tab', function (event) {
        current_house = house;
        $('#house-id').val(house.id);
        $('#house-name').val(house.name);
        $('#house-description').val(house.description);

        const plan_tabs = $('#plan-tabs');
        plan_tabs.empty();

        const plan_tab_template = $('#plan-tab');
        const plan_tab = plan_tab_template[0].content.cloneNode(true);
        plan_tab.id += house.id;
        const plan_tab_content = plan_tab.querySelector('a');
        plan_tab_content.classList.add('active', 'bi', 'bi-house');
        plan_tabs.append(plan_tab);

        const tl_data = new TimelinesData();
        const gr_data = new GraphData();
        context.timelines.set(house.id.toString(), tl_data);
        context.graphs.set(house.id.toString(), gr_data);

        $('#plan-tab-' + house.id).on('show.bs.tab', function (event) {
            current_plan = house.id.toString();

            timelines.update(tl_data);
            timelines.updateTime(tl_data);
            graph.update(gr_data);
        });

        const sensors_list = $('#house-sensors-list');
        sensors_list.empty();
        const sensor_row_template = $('#house-sensor-row');
        const robots_list = $('#house-robots-list');
        robots_list.empty();
        const robot_row_template = $('#house-robot-row');
        for (const c_device of house.devices.sort((a, b) => a.name.localeCompare(b.name)))
            switch (c_device.type.type) {
                case 'sensor':
                    sensors_list.append(create_house_sensor_row(sensor_row_template, c_device));
                    break;
                case 'robot':
                    robots_list.append(create_house_robot_row(robot_row_template, house.id, c_device));
                    refine_house_robot_row(house.id, c_device);
                    const plan_tab = create_plan_tab(plan_tab_template, c_device);
                    plan_tabs.append(plan_tab);
                    break;
                default:
                    console.error('invalid device type ' + c_device.type);
                    break;
            }

        const users_list = $('#house-users-list');
        users_list.empty();
        const user_row_template = $('#house-user-row');
        for (const c_user of house.users.sort((a, b) => (a.lastName + a.firstName).localeCompare(b.lastName + b.firstName)))
            users_list.append(create_house_user_row(user_row_template, c_user));

        $('#house-save').click(function () {
            // we set the device types..
            fetch('http://' + config.host + ':' + config.service_port + '/house_params/?house_id=' + id, {
                method: 'get',
                headers: { 'Authorization': 'Basic ' + context.user.id }
            }).then(response => {
                if (response.ok) {
                    response.text().then(data => { download('params.yaml', data); });
                } else
                    alert(response.statusText);
            });
        });
    });
}

function create_new_house_user_row(template, user) {
    const user_row = template[0].content.cloneNode(true);
    const row_content = user_row.querySelector('.list-group-item');

    const user_input = row_content.querySelector('.user_input');
    user_input.id += user.id;
    user_input.setAttribute('user_id', user.id);

    const user_label = row_content.querySelector('.user_label');
    user_label.htmlFor += user.id;
    user_label.append(user.lastName + ', ' + user.firstName);

    return user_row;
}

function create_house_user_row(template, user) {
    const user_row = template[0].content.cloneNode(true);
    const row_content = user_row.querySelector('.list-group-item');
    row_content.id += user.id;

    const online_span = user_row.querySelector('.user_online');
    online_span.id += user.id;
    online_span.setAttribute('user_id', user.id);
    online_span.classList.add(user.online ? config.online_icon : config.offline_icon);

    user_row.querySelector('.user_name').append(user.lastName + ', ' + user.firstName);
    return row_content;
}

function create_plan_tab(template, robot) {
    const plan_tab = template[0].content.cloneNode(true);
    const plan_tab_content = plan_tab.querySelector('a');
    plan_tab_content.id += robot.id;
    plan_tab_content.append(robot.name);
    return plan_tab;
}

function create_house_sensor_row(template, sensor) {
    const sensor_row = template[0].content.cloneNode(true);
    const row_content = sensor_row.querySelector('.list-group-item');
    row_content.id += sensor.id;
    row_content.append(sensor.name);
    return row_content;
}

function create_house_robot_row(template, house_id, robot) {
    const robot_row = template[0].content.cloneNode(true);
    const row_content = robot_row.querySelector('.list-group-item');
    row_content.id += house_id + '-' + robot.id;
    row_content.append(robot.name);

    const tl_data = new TimelinesData();
    const gr_data = new GraphData();
    context.timelines.set(house_id + '/' + robot.id, tl_data);
    context.graphs.set(house_id + '/' + robot.id, gr_data);

    return row_content;
}

function refine_house_robot_row(house_id, robot) {
    $('#plan-tab-' + house_id + '-' + robot.id).on('show.bs.tab', function (event) {
        current_plan = house_id.toString();

        const c_tl_data = context.timelines.get(house_id + '/' + robot.id);
        const c_gr_data = context.graphs.get(house_id + '/' + robot.id);
        timelines.update(c_tl_data);
        timelines.updateTime(c_tl_data);
        graph.update(c_gr_data);
    });
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