#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "msgs/deliberative_state.h"
#include "msgs/can_start.h"
#include "msgs/start_task.h"

namespace sir
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h), notify_state(handle.advertise<msgs::deliberative_state>("deliberative_state", 10, true)), can_start(h.serviceClient<msgs::can_start>("can_start")), start_task(h.serviceClient<msgs::start_task>("start_task"))
    {
        ROS_DEBUG("Advertising deliberative services..");
        handle.advertiseService("create_reasoner", &deliberative_manager::create_reasoner, this);
        handle.advertiseService("new_requirement", &deliberative_manager::new_requirement, this);
        handle.advertiseService("task_finished", &deliberative_manager::task_finished, this);
    }
    deliberative_manager::~deliberative_manager() {}

    void deliberative_manager::tick()
    {
        for (auto &exec : executors)
            exec.second->get_executor().tick();
    }

    bool deliberative_manager::create_reasoner(msgs::create_reasoner::Request &req, msgs::create_reasoner::Response &res)
    {
        ROS_DEBUG("[%lu] Creating new reasoner..", req.reasoner_id);
        executors[req.reasoner_id] = new deliberative_executor(*this, req.reasoner_id);
        res.created = true;
        return true;
    }

    bool deliberative_manager::new_requirement(msgs::new_requirement::Request &req, msgs::new_requirement::Response &res)
    {
        ROS_DEBUG("[%lu] Adding new requirement..", req.reasoner_id);
        executors.at(req.reasoner_id)->get_solver().read(req.requirement);
        res.consistent = true;
        return true;
    }

    bool deliberative_manager::task_finished(msgs::task_finished::Request &req, msgs::task_finished::Response &res)
    {
        ROS_DEBUG("[%lu] Ending a task..", req.reasoner_id);
        executors.at(req.reasoner_id)->finish_task(req.task_id, req.success);
        res.ended = true;
        return true;
    }
} // namespace sir
