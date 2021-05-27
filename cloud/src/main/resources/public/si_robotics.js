import * as config from './config.js'
import * as context from './context.js'
import * as admin_users from "./admin_users.js";
import * as admin_houses from "./admin_houses.js";
import * as admin_device_types from "./admin_device_types.js";
import { TimelinesData } from './modules/timelines.js';
import { GraphData } from './modules/graph.js';

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

            setup_ws();
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

function setup_ws() {
    ws = new WebSocket('ws://' + config.host + ':' + config.service_port + '/communication/?id=' + context.user.id, 'communication');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'Graph':
                if (!context.graphs.has(c_msg.plan_id))
                    context.graphs.set(c_msg.plan_id, new GraphData());
                context.graphs.get(c_msg.plan_id).reset(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'FlawCreated':
                context.graphs.get(c_msg.plan_id).flaw_created(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'FlawStateChanged':
                context.graphs.get(c_msg.plan_id).flaw_state_changed(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'FlawCostChanged':
                context.graphs.get(c_msg.plan_id).flaw_cost_changed(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'FlawPositionChanged':
                context.graphs.get(c_msg.plan_id).flaw_position_changed(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'CurrentFlaw':
                context.graphs.get(c_msg.plan_id).current_flaw(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'ResolverCreated':
                context.graphs.get(c_msg.plan_id).resolver_created(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'ResolverStateChanged':
                context.graphs.get(c_msg.plan_id).resolver_state_changed(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'CurrentResolver':
                context.graphs.get(c_msg.plan_id).current_resolver(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'CausalLinkAdded':
                context.graphs.get(c_msg.plan_id).causal_link_added(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.graph.update(context.graphs.get(c_msg.plan_id));
                break;
            case 'Timelines':
                if (!context.timelines.has(c_msg.plan_id))
                    context.timelines.set(c_msg.plan_id, new TimelinesData());
                context.timelines.get(c_msg.plan_id).reset(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.timelines.update(context.timelines.get(c_msg.plan_id));
                break;
            case 'Tick':
                context.timelines.get(c_msg.plan_id).tick(c_msg);
                if (admin_houses.current_plan == c_msg.plan_id)
                    admin_houses.timelines.updateTime(context.timelines.get(c_msg.plan_id));
                break;
            case 'StartingAtoms':
                context.timelines.get(c_msg.plan_id).starting_atoms(c_msg);
                break;
            case 'EndingAtoms':
                context.timelines.get(c_msg.plan_id).ending_atoms(c_msg);
                break;
            default:
                console.log(msg);
                break;
        }
        ws.onclose = () => setTimeout(setup_ws, 1000);
    };
}