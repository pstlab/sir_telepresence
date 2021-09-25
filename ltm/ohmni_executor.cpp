#include "ohmni_executor.h"
#include "local_task_manager.h"
#include "predicate.h"
#include "std_msgs/Int8.h"

using namespace ratio;

namespace sir
{
    ohmni_executor::ohmni_executor(local_task_manager &ltm) : ltm(ltm), slv(), exec(slv), core_listener(slv), executor_listener(exec)
    { // we read the domain files..
        slv.read("");
    }
    ohmni_executor::~ohmni_executor() {}

    void ohmni_executor::started_solving()
    {
        ROS_INFO("Started solving..");
        state = Solving;
    }
    void ohmni_executor::solution_found()
    {
        ROS_INFO("Solution found..");
        state = Executing;
    }
    void ohmni_executor::inconsistent_problem()
    {
        ROS_INFO("Inconsistent problem..");
        state = Inconsistent;
    }

    void ohmni_executor::tick(const smt::rational &time)
    {
        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= exec.get_current_time())
            state = Finished;
    }

    void ohmni_executor::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        // exec.dont_start_yet(atms);
    }
    void ohmni_executor::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        for (auto &&atm : atms)
            ROS_INFO("Starting task %s..", atm->get_type().get_name().c_str());
        current_tasks.insert(atms.begin(), atms.end());
    }

    void ohmni_executor::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to finish..
        std::unordered_set<ratio::atom *> dey;
        for (const auto &atm : atms)
            if (current_tasks.count(atm))
                dey.insert(atm);

        if (!dey.empty())
            exec.dont_end_yet(atms);
    }
    void ohmni_executor::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
        for (auto &&atm : atms)
            ROS_INFO("Ending task %s..", atm->get_type().get_name().c_str());
    }
} // namespace sir