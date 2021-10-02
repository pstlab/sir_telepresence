#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "msgs/notify_reasoner_state.h"
#include "msgs/can_start.h"

namespace sir
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h), notify_state(h.serviceClient<msgs::notify_reasoner_state>("notify_reasoner_state")), can_start(h.serviceClient<msgs::can_start>("can_start"))
    {
        handle.advertiseService("create_reasoner", &deliberative_manager::create_reasoner, this);
        handle.advertiseService("new_requirement", &deliberative_manager::new_requirement, this);
        handle.advertiseService("task_finished", &deliberative_manager::task_finished, this);
    }
    deliberative_manager::~deliberative_manager() {}

    bool deliberative_manager::create_reasoner(msgs::create_reasoner::Request &req, msgs::create_reasoner::Response &res)
    {
        executors[req.reasoner_id] = new deliberative_executor(*this, req.reasoner_id);
        res.created = true;
        return true;
    }

    bool deliberative_manager::new_requirement(msgs::new_requirement::Request &req, msgs::new_requirement::Response &res)
    {
        executors.at(req.reasoner_id)->get_solver().read(req.requirement);
        res.consistent = true;
        return true;
    }

    bool deliberative_manager::task_finished(msgs::task_finished::Request &req, msgs::task_finished::Response &res)
    {
        executors.at(req.reasoner_id)->finish_task(req.task_id, req.success);
        res.ended = true;
        return true;
    }
} // namespace sir
