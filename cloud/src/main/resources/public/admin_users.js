import * as config from './config.js'
import * as context from './context.js'
import * as admin_houses from "./admin_houses.js";

export function init() {
    // we set the users..
    fetch('http://' + config.host + ':' + config.service_port + '/users', {
        method: 'get',
        headers: { 'Authorization': 'Basic ' + context.user.id }
    }).then(response => {
        if (response.ok) {
            response.json().then(data => {
                const users_list = $('#users-list');
                const user_row_template = $('#user-row');
                for (const c_user of data.sort((a, b) => (a.lastName + a.firstName).localeCompare(b.lastName + b.firstName))) {
                    context.users.set(c_user.id, c_user);
                    if (c_user.id != context.user.id) {
                        const user_row = create_user_row(user_row_template, c_user);
                        users_list.append(user_row);
                    }
                    admin_houses.new_user(c_user);
                }
            });
        } else
            alert(response.statusText);
    });
}

function create_user_row(template, user) {
    const user_row = template[0].content.cloneNode(true);

    const row_content = user_row.querySelector('.list-group-item');
    row_content.id += user.id;

    const online_span = user_row.querySelector('.user_online');
    online_span.id += user.id;
    online_span.setAttribute('user_id', user.id);
    online_span.classList.add(user.online ? online_icon : offline_icon);

    const user_name = user_row.querySelector('.user_name');
    user_name.append(user.lastName + ', ' + user.firstName);
    return row_content;
}