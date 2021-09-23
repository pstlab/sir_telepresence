#include "local_task_manager.h"

namespace sir
{
    local_task_manager::local_task_manager() : slv(), exec(slv), ratio::executor_listener(exec) {}
    local_task_manager::~local_task_manager() {}

    void local_task_manager::starting(const std::unordered_set<ratio::atom *> &atms)
    { // tell the executor to do not start some atoms..
        // exec.dont_start_yet(atms);
    }
    void local_task_manager::start(const std::unordered_set<ratio::atom *> &atms)
    { // these atoms are now started..
    }

    void local_task_manager::ending(const std::unordered_set<ratio::atom *> &atms)
    { // tell the executor to do not start some atoms..
        // exec.dont_end_yet(atms);
    }
    void local_task_manager::end(const std::unordered_set<ratio::atom *> &atms)
    { // these atoms are now ended..
    }
} // namespace sir