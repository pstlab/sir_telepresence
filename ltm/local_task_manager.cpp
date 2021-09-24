#include "local_task_manager.h"
#include "ohmni_executor.h"

namespace sir
{

    local_task_manager::local_task_manager(ros::NodeHandle &handle) : handle(handle) {}
    local_task_manager::~local_task_manager() {}

    void local_task_manager::tick()
    {
        if (!exec) // we create a new plan..
            create_new_plan();
        else if (exec->get_solver_state() == Finished || exec->get_solver_state() == Inconsistent)
        { // we make some cleanings before creating a new plan..
            delete exec;
            create_new_plan();
        }
        else if (exec->get_solver_state() == Executing)
            // TODO: the executor's tick should be executed on a separate thread..
            exec->get_executor().tick();
    }

    void local_task_manager::create_new_plan()
    {
        exec = new ohmni_executor();
        // TODO: create the planning problem..
        // TODO: solve the planning problem on a separate thread..
    }
} // namespace sir