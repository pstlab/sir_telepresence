import * as config from './config.js'
import * as context from './context.js'
import * as admin_houses from "./admin_houses.js";

export function init() {
    // we set the device types..
    fetch('http://' + config.host + ':' + config.service_port + '/device_types', {
        method: 'get',
        headers: { 'Authorization': 'Basic ' + context.user.id }
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
                            sensor_types_list.append(create_sensor_type_row(sensor_type_row_template, c_device_type));
                            admin_houses.new_sensor_type(c_device_type);
                            break;
                        case 'robot':
                            robot_types_list.append(create_robot_type_row(robot_type_row_template, c_device_type));
                            admin_houses.new_robot_type(c_device_type);
                            break;
                        default:
                            console.error('invalid device type ' + c_device_type.type);
                            break;
                    }
            });
        } else
            alert(response.statusText);
    });
}

export function new_sensor_type() {
    const form = new FormData();
    form.append('name', $('#new-sensor-type-name').val());
    form.append('description', $('#new-sensor-type-description').val());
    form.append('category', 0);
    fetch('http://' + config.host + ':' + config.service_port + '/sensor_type', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + context.user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-sensor-type-name').val('');
            $('#new-sensor-type-description').val('');
            $('#new-sensor-type-category').val('');
            response.json().then(sensor_type => {
                $('#new-house-sensor-type').append($('<option>', { value: sensor_type.id, text: sensor_type.name }));
                $('#sensor-types-list').append(create_sensor_type_row($('#sensor-type-row'), sensor_type));
            });
        } else
            alert(response.statusText);
    });
}

export function new_robot_type() {
    const form = new FormData();
    form.append('name', $('#new-robot-type-name').val());
    form.append('description', $('#new-robot-type-description').val());
    form.append('category', $('#new-robot-type-category').val());
    form.append('configuration', $('#new-robot-type-configuration').val());
    fetch('http://' + config.host + ':' + config.service_port + '/robot_type', {
        method: 'post',
        headers: { 'Authorization': 'Basic ' + context.user.id },
        body: form
    }).then(response => {
        if (response.ok) {
            $('#new-robot-type-name').val('');
            $('#new-robot-type-description').val('');
            $('#new-robot-type-category').val('');
            $('#new-robot-type-configuration').val('');
            response.json().then(robot_type => {
                $('#new-house-robot-type').append($('<option>', { value: robot_type.id, text: robot_type.name }));
                $('#robot-types-list').append(create_robot_type_row($('#robot-type-row'), robot_type));
            });
        } else
            alert(response.statusText);
    });
}

function create_sensor_type_row(template, sensor_type) {
    const sensor_type_row = template[0].content.cloneNode(true);
    const row_content = sensor_type_row.querySelector('.list-group-item');
    row_content.id += sensor_type.id;
    row_content.append(sensor_type.name);
    return sensor_type_row;
}

function create_robot_type_row(template, robot_type) {
    const robot_type_row = template[0].content.cloneNode(true);
    const row_content = robot_type_row.querySelector('.list-group-item');
    row_content.id += robot_type.id;
    row_content.append(robot_type.name);
    return robot_type_row;
}