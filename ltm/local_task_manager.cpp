#include "local_task_manager.h"
#include "std_msgs/Int8.h"

using namespace ratio;

namespace sir
{
    local_task_manager::local_task_manager(ros::NodeHandle &handle) : handle(handle), slv(), exec(slv), core_listener(slv), executor_listener(exec) {}
    local_task_manager::~local_task_manager() {}

    void local_task_manager::started_solving() {}
    void local_task_manager::solution_found() {}
    void local_task_manager::inconsistent_problem() {}

    void local_task_manager::tick(const smt::rational time)
    {
        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= time)
        { // the plan is now useless..
        }
    }

    void local_task_manager::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor to do not start some atoms..
        // exec.dont_start_yet(atms);
    }
    void local_task_manager::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
    }

    void local_task_manager::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor to do not start some atoms..
        // exec.dont_end_yet(atms);
    }
    void local_task_manager::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
    }
} // namespace sir