#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "atom.h"
#include "msgs/deliberative_state.h"
#include "msgs/can_start.h"
#include "msgs/start_task.h"
#include <ros/ros.h>

using namespace ratio;

namespace sir
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), core_listener(slv), executor_listener(exec)
    {
        // we read the domain files..
        slv.read("class Dialogue : StateVariable { predicate Configuring() { duration >= 20.0; } } class Ohmni { Dialogue dialogue = new Dialogue(); } Ohmni ohmni = new Ohmni();");
        ROS_DEBUG("[%lu] Created reasoner..", reasoner_id);
        set_state(msgs::deliberative_state::idle);
    }
    deliberative_executor::~deliberative_executor() {}

    void deliberative_executor::started_solving()
    {
        ROS_DEBUG("[%lu] Started reasoning..", reasoner_id);
        state = Reasoning;
        set_state(msgs::deliberative_state::reasoning);
    }
    void deliberative_executor::solution_found()
    {
        ROS_DEBUG("[%lu] Solution found..", reasoner_id);
        state = Executing;
        set_state(msgs::deliberative_state::executing);
    }
    void deliberative_executor::inconsistent_problem()
    {
        ROS_DEBUG("[%lu] Inconsistent problem..", reasoner_id);
        state = Inconsistent;
        set_state(msgs::deliberative_state::inconsistent);
    }

    void deliberative_executor::tick(const smt::rational &time)
    {
        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= exec.get_current_time())
        {
            ROS_DEBUG("[%lu] Exhausted plan..", reasoner_id);
            state = Finished;
            set_state(msgs::deliberative_state::finished);
        }
    }

    void deliberative_executor::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        std::unordered_set<ratio::atom *> dsy;
        msgs::can_start srv;
        task t;
        for (const auto &atm : atms)
        {
            t = to_task(*atm);
            srv.request.task_name = t.task_name;
            srv.request.par_names = t.par_names;
            srv.request.par_values = t.par_values;
            if (d_mngr.can_start.call(srv) && !srv.response.can_start)
                dsy.insert(atm);
        }

        if (!dsy.empty())
            exec.dont_start_yet(atms);
    }
    void deliberative_executor::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        msgs::start_task srv;
        task t;
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Starting task %s..", reasoner_id, atm->get_type().get_name().c_str());
            t = to_task(*atm);
            srv.request.reasoner_id = reasoner_id;
            srv.request.task_id = t.task_id;
            srv.request.task_name = t.task_name;
            srv.request.par_names = t.par_names;
            srv.request.par_values = t.par_values;
            if (d_mngr.can_start.call(srv) && srv.response.started)
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
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Ending task %s..", reasoner_id, atm->get_type().get_name().c_str());
        }
    }

    void deliberative_executor::finish_task(const smt::var &id, const bool &success)
    {
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }

    void deliberative_executor::set_state(const unsigned int &state)
    {
        msgs::deliberative_state state_msg;
        state_msg.reasoner_id = reasoner_id;
        state_msg.reasoner_state = state;
        d_mngr.notify_state.publish(state_msg);
    }

    deliberative_executor::task deliberative_executor::to_task(const ratio::atom &atm) const
    {
        uint64_t task_id = atm.get_sigma();
        std::string task_name = atm.get_type().get_name();
        std::vector<std::string> par_names;
        std::vector<std::string> par_values;
        for (const auto &xpr : atm.get_exprs())
            if (!xpr.first.compare(START) && !xpr.first.compare(END) && !xpr.first.compare(AT) && !xpr.first.compare(TAU))
            {
                par_names.push_back(xpr.first);
                if (bool_item *bi = dynamic_cast<bool_item *>(&*xpr.second))
                    switch (atm.get_core().bool_value(bi))
                    {
                    case smt::True:
                        par_values.push_back("true");
                        break;
                    case smt::False:
                        par_values.push_back("false");
                        break;
                    default:
                        break;
                    }
                else if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr.second))
                    par_values.push_back(to_string(atm.get_core().arith_value(ai).get_rational()));
                else if (string_item *si = dynamic_cast<string_item *>(&*xpr.second))
                    par_values.push_back(si->get_value());
            }
        return {task_id, task_name, par_names, par_values};
    }
} // namespace sir