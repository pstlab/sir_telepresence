#include "sequencer.h"
#include "msgs/create_reasoner.h"
#include "msgs/destroy_reasoner.h"
#include "msgs/new_requirement.h"
#include "msgs/task_finished.h"
#include <ros/package.h>

namespace sir
{
    std::string system_to_string(unsigned int &system_state);
    std::string deliberative_to_string(const std::map<uint64_t, unsigned int> &deliberative_state);
    std::string navigation_to_string(unsigned int &navigation_state);
    std::string dialogue_to_string(unsigned int &dialogue_state);

    sequencer::sequencer(ros::NodeHandle &h) : handle(h),
                                               notify_state(h.advertise<msgs::system_state>("system_state", 10, true)),
                                               deliberative_state_sub(h.subscribe("deliberative_state", 100, &sequencer::updated_deliberative_state, this)),
                                               navigation_state_sub(h.subscribe("navigation_state", 100, &sequencer::updated_navigation_state, this)),
                                               dialogue_state_sub(h.subscribe("dialogue_state", 100, &sequencer::updated_dialogue_state, this)),
                                               create_reasoner(h.serviceClient<msgs::create_reasoner>("create_reasoner")),
                                               destroy_reasoner(h.serviceClient<msgs::destroy_reasoner>("destroy_reasoner")),
                                               new_requirement(h.serviceClient<msgs::new_requirement>("new_requirement")),
                                               task_finished(h.serviceClient<msgs::task_finished>("task_finished")),
                                               can_start_server(h.advertiseService("can_start", &sequencer::can_start, this)),
                                               start_task_server(h.advertiseService("start_task", &sequencer::start_task, this)),
                                               start_physical_exercise_task(h.serviceClient<msgs::start_task>("start_physical_exercise")),
                                               start_dialogue_task(h.serviceClient<msgs::start_task>("start_dialogue_task"))
    {
        create_reasoner.waitForExistence();
        new_requirement.waitForExistence();
        task_finished.waitForExistence();
        start_physical_exercise_task.waitForExistence();
        start_dialogue_task.waitForExistence();

        ros::service::waitForService("set_face");
    }
    sequencer::~sequencer() {}

    void sequencer::tick()
    {
        ROS_DEBUG("{\"System\": %s, \"Deliberative\": %s, \"Navigation\": %s, \"Dialogue\": %s}", system_to_string(system_state).c_str(), deliberative_to_string(deliberative_state).c_str(), navigation_to_string(navigation_state).c_str(), dialogue_to_string(dialogue_state).c_str());

        switch (system_state)
        {
        case msgs::system_state::unconfigured:
        { // we start a configuration plan..
            ROS_INFO("Starting system configuration..");
            set_state(msgs::system_state::configuring);

            const uint64_t reasoner_id = 0;
            deliberative_state[reasoner_id] = msgs::deliberative_state::idle;
            msgs::create_reasoner new_reasoner;
            new_reasoner.request.reasoner_id = reasoner_id;

            std::vector<std::string> domain_files;
            ros::param::get("~domain_files", domain_files);
            std::string package_path = ros::package::getPath("sequencer_tier") + '/';
            for (const auto &file : domain_files)
                new_reasoner.request.domain_files.push_back(package_path + file);
            std::string config_goal;
            ros::param::get("~config_goal", config_goal);
            new_reasoner.request.requirements.push_back(config_goal);
            std::vector<std::string> notify_start;
            ros::param::get("~notify_start", new_reasoner.request.notify_start);

            create_reasoner.call(new_reasoner);
            break;
        }
        case msgs::system_state::configuring:
        { // we are configuring the system..
            const uint64_t configure_reasoner_id = 0;
            switch (deliberative_state.at(configure_reasoner_id))
            {
            case msgs::deliberative_state::finished:
            {
                ROS_INFO("System configured..");
                msgs::destroy_reasoner dest_reasoner;
                dest_reasoner.request.reasoner_id = configure_reasoner_id;
                destroy_reasoner.call(dest_reasoner);
                set_state(msgs::system_state::configured);
                deliberative_state.erase(configure_reasoner_id);
                break;
            }
            default:
                break;
            }
            break;
        }
        case msgs::system_state::configured:
        { // we start the default plan..
            ROS_INFO("Starting default plan..");
            set_state(msgs::system_state::running);

            const uint64_t reasoner_id = 0;
            deliberative_state[reasoner_id] = msgs::deliberative_state::idle;
            msgs::create_reasoner new_reasoner;
            new_reasoner.request.reasoner_id = reasoner_id;

            std::vector<std::string> domain_files;
            ros::param::get("~domain_files", domain_files);
            std::string package_path = ros::package::getPath("sequencer_tier") + '/';
            for (const auto &file : domain_files)
                new_reasoner.request.domain_files.push_back(package_path + file);
            std::string running_goal;
            ros::param::get("~running_goal", running_goal);
            new_reasoner.request.requirements.push_back(running_goal);
            std::vector<std::string> notify_start;
            ros::param::get("~notify_start", new_reasoner.request.notify_start);

            create_reasoner.call(new_reasoner);
            break;
        }
        default:
            break;
        }
    }

    bool sequencer::can_start(msgs::can_start::Request &req, msgs::can_start::Response &res)
    {
        ROS_ASSERT(req.par_names.size() == req.par_values.size());
        ROS_DEBUG("checking whether task \'%s\' can start..", req.task_name.c_str());
        if (req.task_name == "Interacting")
            res.can_start = dialogue_state == msgs::dialogue_state::idle;
        else
        {
            ROS_WARN("Unknown task name: %s", req.task_name.c_str());
            res.can_start = false;
        }
        return true;
    }

    bool sequencer::start_task(msgs::start_task::Request &req, msgs::start_task::Response &res)
    {
        ROS_ASSERT(req.par_names.size() == req.par_values.size());
        ROS_INFO("starting task \'%s\'..", req.task_name.c_str());
        if (req.task_name == "Interacting")
        { // starts an interaction with the user..
            msgs::start_task sd_srv;
            sd_srv.request.reasoner_id = req.reasoner_id;
            sd_srv.request.task_id = req.task_id;
            for (size_t i = 0; i < req.par_names.size(); i++)
                if (req.par_names.at(i) == "intent")
                    sd_srv.request.task_name = req.par_values.at(i);
                else
                {
                    sd_srv.request.par_names.push_back(req.par_names.at(i));
                    sd_srv.request.par_values.push_back(req.par_values.at(i));
                }
            res.started = start_dialogue_task.call(sd_srv);
        }
        else if (req.task_name == "BicepsCurl")
        { // starts a count the biceps curl physical exercise with the user..
            msgs::start_task bc_srv;
            bc_srv.request.reasoner_id = req.reasoner_id;
            bc_srv.request.task_id = req.task_id;
            bc_srv.request.task_name = "BICEPS CURL";
            for (size_t i = 0; i < req.par_names.size(); i++)
            {
                bc_srv.request.par_names.push_back(req.par_names.at(i));
                bc_srv.request.par_values.push_back(req.par_values.at(i));
            }
            res.started = start_physical_exercise_task.call(bc_srv);
        }
        else if (req.task_name == "CountTheWord")
        { // starts a count the word cognitive exercise with the user..
            msgs::start_task ctw_srv;
            ctw_srv.request.reasoner_id = req.reasoner_id;
            ctw_srv.request.task_id = req.task_id;
            ctw_srv.request.task_name = "start_count_the_word_cognitive_exercise";
            for (size_t i = 0; i < req.par_names.size(); i++)
            {
                ctw_srv.request.par_names.push_back(req.par_names.at(i));
                ctw_srv.request.par_values.push_back(req.par_values.at(i));
            }
            res.started = start_dialogue_task.call(ctw_srv);
        }
        else
        {
            ROS_ERROR("Unknown task name: %s", req.task_name.c_str());
            res.started = false;
        }
        return true;
    }

    void sequencer::set_state(const unsigned int &state)
    {
        system_state = state;
        msgs::system_state state_msg;
        state_msg.system_state = state;
        notify_state.publish(state_msg);
    }

    std::string system_to_string(unsigned int &system_state)
    {
        switch (system_state)
        {
        case msgs::system_state::unconfigured:
            return "\"Unconfigured\"";
        case msgs::system_state::configuring:
            return "\"Configuring\"";
        case msgs::system_state::configured:
            return "\"Configured\"";
        case msgs::system_state::running:
            return "\"Running\"";
        default:
            return "\"-\"";
        }
    }

    std::string deliberative_to_string(const std::map<uint64_t, unsigned int> &deliberative_state)
    {
        std::string delib_state;
        for (const auto &r : deliberative_state)
        {
            if (r.first != deliberative_state.begin()->first)
                delib_state.append(" ");
            delib_state.append("(").append(std::to_string(r.first)).append(") ");
            switch (r.second)
            {
            case msgs::deliberative_state::idle:
                delib_state.append("\"Idle\"");
                break;
            case msgs::deliberative_state::reasoning:
                delib_state.append("\"Reasoning\"");
                break;
            case msgs::deliberative_state::executing:
                delib_state.append("\"Executing\"");
                break;
            case msgs::deliberative_state::finished:
                delib_state.append("\"Finished\"");
                break;
            case msgs::deliberative_state::inconsistent:
                delib_state.append("\"Inconsistent\"");
                break;
            default:
                delib_state.append("\"-\"");
                break;
            }
        }
        return delib_state;
    }

    std::string navigation_to_string(unsigned int &navigation_state)
    {
        switch (navigation_state)
        {
        case msgs::navigation_state::idle:
            return "\"Idle\"";
        case msgs::navigation_state::navigating:
            return "\"Navigating\"";
        default:
            return "\"-\"";
        }
    }

    std::string dialogue_to_string(unsigned int &dialogue_state)
    {
        switch (dialogue_state)
        {
        case msgs::dialogue_state::idle:
            return "\"Idle\"";
        case msgs::dialogue_state::listening:
            return "\"Listening\"";
        case msgs::dialogue_state::speaking:
            return "\"Speaking\"";
        default:
            return "\"-\"";
        }
    }
} // namespace sir
