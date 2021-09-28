#include "dialogue_manager.h"
#include "local_task_manager.h"
#include "predicate.h"
#include "msgs/start_dialogue.h"

using namespace ratio;

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
            ROS_DEBUG("Started profile gathering phase..");
            d_state = Talking;
        }
        else
        {
            ROS_ERROR("Cannot start profile gathering phase..");
        }
    }

    void dialogue_manager::start_dialogue(const atom &atm)
    {
        ROS_ASSERT(current_dialogue == nullptr);
        ROS_INFO("Starting dialogue %s..", atm.get_type().get_name().c_str());
        msgs::start_dialogue srv;
        srv.request.dialogue_id = atm.get_sigma();
        auto xprs = atm.get_exprs();

        string_expr intent_xpr = xprs.at("intent");
        srv.request.intent.assign((*intent_xpr).get_value());

        for (const auto &xpr : xprs)
            if (!xpr.first.compare("intent") && !xpr.first.compare(START) && !xpr.first.compare(END) && !xpr.first.compare(AT) && !xpr.first.compare(TAU))
            {
                srv.request.entity_names.push_back(xpr.first);
                if (bool_item *bi = dynamic_cast<bool_item *>(&*xpr.second))
                    switch (atm.get_core().bool_value(bi))
                    {
                    case smt::True:
                        srv.request.entity_values.push_back("true");
                        break;
                    case smt::False:
                        srv.request.entity_values.push_back("false");
                        break;
                    default:
                        break;
                    }
                else if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr.second))
                    srv.request.entity_values.push_back(to_string(atm.get_core().arith_value(ai).get_rational()));
                else if (string_item *si = dynamic_cast<string_item *>(&*xpr.second))
                    srv.request.entity_values.push_back(si->get_value());
            }

        if (service_client.call(srv))
        {
            ROS_DEBUG("Started dialogue %s..", atm.get_type().get_name().c_str());
            d_state = Talking;
            current_dialogue = &atm;
        }
        else
        {
            ROS_ERROR("Cannot start dialogue %s..", atm.get_type().get_name().c_str());
        }
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
