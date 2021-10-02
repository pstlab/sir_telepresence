#include "deliberative_executor.h"
#include "predicate.h"
#include "atom.h"
#include "msgs/notify_reasoner_state.h"
#include <ros/ros.h>

using namespace ratio;

namespace sir
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const size_t &id) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), core_listener(slv), executor_listener(exec)
    {
        // we read the domain files..
        slv.read("");
    }
    deliberative_executor::~deliberative_executor() {}

    void deliberative_executor::started_solving()
    {
        ROS_INFO("Started solving..");
        state = Solving;
        msgs::notify_reasoner_state srv;
    }
    void deliberative_executor::solution_found()
    {
        ROS_INFO("Solution found..");
        state = Executing;
    }
    void deliberative_executor::inconsistent_problem()
    {
        ROS_INFO("Inconsistent problem..");
        state = Inconsistent;
    }

    void deliberative_executor::tick(const smt::rational &time)
    {
        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= exec.get_current_time())
            state = Finished;
    }

    void deliberative_executor::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        // exec.dont_start_yet(atms);
    }
    void deliberative_executor::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        for (const auto &atm : atms)
        {
            ROS_INFO("Starting task %s..", atm->get_type().get_name().c_str());
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
            ROS_INFO("Ending task %s..", atm->get_type().get_name().c_str());
    }

    void deliberative_executor::finish_task(const smt::var &id, const bool &success)
    {
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }
} // namespace sir