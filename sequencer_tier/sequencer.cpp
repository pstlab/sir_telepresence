#include "sequencer.h"
#include "msgs/start_task.h"

namespace sir
{
    const char *system_to_string(unsigned int &system_state);
    const char *deliberative_to_string(unsigned int &deliberative_state);
    const char *dialogue_to_string(unsigned int &dialogue_state);

    sequencer::sequencer(ros::NodeHandle &handle) : handle(handle), notify_state(handle.advertise<msgs::system_state>("system_state", 10, true)), system_state_sub(handle.subscribe("system_state", 100, &sequencer::updated_system_state, this)), deliberative_state_sub(handle.subscribe("deliberative_state", 100, &sequencer::updated_deliberative_state, this)), dialogue_state_sub(handle.subscribe("dialogue_state", 100, &sequencer::updated_dialogue_state, this)), start_dialogue_service(handle.serviceClient<msgs::start_task>("start_dialogue")) {}
    sequencer::~sequencer() {}

    void sequencer::tick()
    {
        ROS_INFO("{\"System\": %s, \"Deliberative\": %s, \"Dialogue\": %s}", system_to_string(system_state), deliberative_to_string(deliberative_state), dialogue_to_string(dialogue_state));

        switch (system_state)
        {
        case msgs::system_state::unconfigured:
            set_state(msgs::system_state::configuring);
            // TODO: create configuration planning problem..
            break;
        default:
            break;
        }
    }

    void sequencer::set_state(const unsigned int &state)
    {
        msgs::system_state state_msg;
        state_msg.system_state = state;
        notify_state.publish(state_msg);
    }

    const char *system_to_string(unsigned int &system_state)
    {
        switch (system_state)
        {
        case msgs::system_state::unconfigured:
            return "\"Unconfigured\"";
        case msgs::system_state::configuring:
            return "\"Configuring\"";
        case msgs::system_state::configured:
            return "\"Configured\"";
        default:
            return "\"-\"";
        }
    }

    const char *deliberative_to_string(unsigned int &deliberative_state)
    {
        switch (deliberative_state)
        {
        case msgs::deliberative_state::idle:
            return "\"Idle\"";
        case msgs::deliberative_state::reasoning:
            return "\"Reasoning\"";
        case msgs::deliberative_state::executing:
            return "\"Executing\"";
        case msgs::deliberative_state::finished:
            return "\"Finished\"";
        case msgs::deliberative_state::inconsistent:
            return "\"Inconsistent\"";
        default:
            return "\"-\"";
        }
    }

    const char *dialogue_to_string(unsigned int &dialogue_state)
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
