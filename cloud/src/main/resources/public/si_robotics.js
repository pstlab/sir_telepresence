import * as config from './config.js'
import * as context from './context.js'
import * as admin_users from "./admin_users.js";
import * as admin_houses from "./admin_houses.js";
import * as admin_device_types from "./admin_device_types.js";

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
                response.json().then(data => { set_user(data); });
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

function set_user(usr) {
    context.set_user(usr);
    $('#body-guest').remove();
    if (context.user.roles.includes('Admin')) {
        $.get('body_admin.html', function (data) {
            $('#sirobotics-body').append(data);
            $('#account-menu').text(context.user.firstName);
            $('#profile-email').val(context.user.email);
            $('#profile-first-name').val(context.user.firstName);
            $('#profile-last-name').val(context.user.lastName);

            $('#users').load('admin_users.html', () => {
                admin_users.init();
            });

            $('#houses').load('admin_houses.html', () => {
                admin_houses.init();
            });

            $('#device-types').load('admin_device_types.html', () => {
                admin_device_types.init();
            });
        });
    }
    else if (context.user.roles.includes('User')) {
        $.get('body_user.html', function (data) {
            $('#sirobotics-body').append(data);
            $('#account-menu').text(context.user.firstName);
            $('#profile-email').val(context.user.email);
            $('#profile-first-name').val(context.user.firstName);
            $('#profile-last-name').val(context.user.lastName);
        });
    }
}