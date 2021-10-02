#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "atom.h"
#include "msgs/notify_reasoner_state.h"
#include "msgs/can_start.h"
#include <ros/ros.h>

using namespace ratio;

namespace sir
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), core_listener(slv), executor_listener(exec)
    {
        // we read the domain files..
        slv.read("");
        ROS_DEBUG("[%lu] Created reasoner..", reasoner_id);
        msgs::notify_reasoner_state srv;
        srv.request.reasoner_id = reasoner_id;
        srv.request.reasoner_state = srv.request.idle;
        d_mngr.notify_state.call(srv);
    }
    deliberative_executor::~deliberative_executor() {}

    void deliberative_executor::started_solving()
    {
        ROS_DEBUG("[%lu] Started reasoning..", reasoner_id);
        state = Reasoning;
        msgs::notify_reasoner_state srv;
        srv.request.reasoner_id = reasoner_id;
        srv.request.reasoner_state = srv.request.reasoning;
        d_mngr.notify_state.call(srv);
    }
    void deliberative_executor::solution_found()
    {
        ROS_DEBUG("[%lu] Solution found..", reasoner_id);
        state = Executing;
        msgs::notify_reasoner_state srv;
        srv.request.reasoner_id = reasoner_id;
        srv.request.reasoner_state = srv.request.executing;
        d_mngr.notify_state.call(srv);
    }
    void deliberative_executor::inconsistent_problem()
    {
        ROS_DEBUG("[%lu] Inconsistent problem..", reasoner_id);
        state = Inconsistent;
        msgs::notify_reasoner_state srv;
        srv.request.reasoner_id = reasoner_id;
        srv.request.reasoner_state = srv.request.inconsistent;
        d_mngr.notify_state.call(srv);
    }

    void deliberative_executor::tick(const smt::rational &time)
    {
        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= exec.get_current_time())
            state = Finished;
    }

    void deliberative_executor::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        std::unordered_set<ratio::atom *> dsy;
        msgs::can_start srv;
        for (const auto &atm : atms)
        {
            srv.request.task_name = atm->get_type().get_name();
            if (d_mngr.can_start.call(srv) && !srv.response.can_start)
                dsy.insert(atm);
        }

        if (!dsy.empty())
            exec.dont_start_yet(atms);
    }
    void deliberative_executor::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Starting task %s..", reasoner_id, atm->get_type().get_name().c_str());
            current_tasks.emplace(atm->get_sigma(), atm);
        }
    }

    void deliberative_executor::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to finish..
        std::unordered_set<ratio::atom *> dey;
        for (const auto &atm : atms)
            if (current_tasks.count(atm->get_sigma()))
                dey.insert(atm);

        if (!dey.empty())
            exec.dont_end_yet(atms);
    }
    void deliberative_executor::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
        for (auto &&atm : atms)
            ROS_DEBUG("[%lu] Ending task %s..", reasoner_id, atm->get_type().get_name().c_str());
    }

    void deliberative_executor::finish_task(const smt::var &id, const bool &success)
    {
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }
} // namespace sir