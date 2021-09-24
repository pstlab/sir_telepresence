#include "ohmni_executor.h"
#include "std_msgs/Int8.h"

using namespace ratio;

namespace sir
{
    ohmni_executor::ohmni_executor() : slv(), exec(slv), core_listener(slv), executor_listener(exec) {}
    ohmni_executor::~ohmni_executor() {}

    void ohmni_executor::started_solving() { state = Solving; }
    void ohmni_executor::solution_found() { state = Executing; }
    void ohmni_executor::inconsistent_problem() { state = Inconsistent; }

    void ohmni_executor::tick(const smt::rational &time)
    {
        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= exec.get_current_time())
            state = Finished;
    }

    void ohmni_executor::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor to do not start some atoms..
        // exec.dont_start_yet(atms);
    }
    void ohmni_executor::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
    }

    void ohmni_executor::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor to do not start some atoms..
        // exec.dont_end_yet(atms);
    }
    void ohmni_executor::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
    }
} // namespace sir