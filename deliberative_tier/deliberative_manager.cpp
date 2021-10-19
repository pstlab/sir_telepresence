#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "msgs/deliberative_state.h"
#include "msgs/can_start.h"
#include "msgs/start_task.h"

namespace sir
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h),
                                                                     create_reasoner_server(h.advertiseService("create_reasoner", &deliberative_manager::create_reasoner, this)),
                                                                     new_requirement_server(h.advertiseService("new_requirement", &deliberative_manager::new_requirement, this)),
                                                                     task_finished_server(h.advertiseService("task_finished", &deliberative_manager::task_finished, this)),
                                                                     notify_state(handle.advertise<msgs::deliberative_state>("deliberative_state", 10, true)),
                                                                     can_start(h.serviceClient<msgs::can_start>("can_start")),
                                                                     start_task(h.serviceClient<msgs::start_task>("start_task")) {}
    deliberative_manager::~deliberative_manager() {}

    void deliberative_manager::tick()
    {
        for (auto &req : pending_requirements)
            while (!req.second.empty())
            {
                const std::string c_req = req.second.front();
                ROS_DEBUG("[%lu] reading:%s", req.first, c_req.c_str());
                executors.at(req.first)->get_solver().read(c_req);
                ROS_DEBUG("[%lu] solving..", req.first);
                executors.at(req.first)->get_solver().solve();
                ROS_DEBUG("[%lu] solution found..", req.first);
                req.second.pop();
            }
        for (auto &exec : executors)
            exec.second->get_executor().tick();
    }

    bool deliberative_manager::create_reasoner(msgs::create_reasoner::Request &req, msgs::create_reasoner::Response &res)
    {
        ROS_DEBUG("Creating new reasoner %lu..", req.reasoner_id);
        executors[req.reasoner_id] = new deliberative_executor(*this, req.reasoner_id);
        res.created = true;
        return true;
    }

    bool deliberative_manager::new_requirement(msgs::new_requirement::Request &req, msgs::new_requirement::Response &res)
    {
        ROS_DEBUG("Adding new requirement to reasoner %lu..", req.reasoner_id);
        pending_requirements[req.reasoner_id].push(req.requirement);
        res.consistent = true;
        return true;
    }

    bool deliberative_manager::task_finished(msgs::task_finished::Request &req, msgs::task_finished::Response &res)
    {
        ROS_DEBUG("Ending task %lu for reasoner %lu..", req.reasoner_id, req.task_id);
        executors.at(req.reasoner_id)->finish_task(req.task_id, req.success);
        res.ended = true;
        return true;
    }
} // namespace sir
