#include "dialogue_manager.h"
#include "local_task_manager.h"
#include "predicate.h"
#include "msgs/start_dialogue.h"

namespace sir
{
    dialogue_manager::dialogue_manager(local_task_manager &ltm) : ltm(ltm), service_client(ltm.get_handle().serviceClient<msgs::start_dialogue>("start_dialogue")) { ltm.get_handle().advertiseService("dialogue_finished", &dialogue_manager::dialogue_finished, this); }
    dialogue_manager::~dialogue_manager() {}

    void dialogue_manager::gather_profile()
    {
        ROS_INFO("Starting profile gathering phase..");
        msgs::start_dialogue srv;
        srv.request.intent = "gather_profile";
        if (service_client.call(srv))
        {
            ROS_INFO("Started profile gathering phase..");
            d_state = Talking;
        }
        else
        {
            ROS_INFO("Cannot start profile gathering phase..");
        }
    }

    void dialogue_manager::start_dialogue(const ratio::atom &atm)
    {
        ROS_ASSERT(current_dialogue == nullptr);
        ROS_INFO("Starting dialogue %s..", atm.get_type().get_name().c_str());
        d_state = Talking;
        current_dialogue = &atm;
    }

    bool dialogue_manager::dialogue_finished(msgs::dialogue_finished::Request &req, msgs::dialogue_finished::Response &res)
    {
        ROS_INFO("Dialogue %lu is finished..", req.dialogue_id);
        d_state = Silent;
        if (current_dialogue)
        { // the dialogue was started by a deliberative initiative..
            ltm.get_executor()->finish_task(req.dialogue_id, !req.dialogue_result);
            current_dialogue = nullptr;
        }
        res.result_code = 0;
        return true;
    }
} // namespace sir
