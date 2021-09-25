#include "local_task_manager.h"
#include "ohmni_executor.h"
#include <thread>

using namespace ratio;

namespace sir
{

    local_task_manager::local_task_manager(ros::NodeHandle &handle) : handle(handle) {}
    local_task_manager::~local_task_manager() {}

    void local_task_manager::tick()
    {
        ROS_INFO("{\"profile\": \"%s\", \"solver\": \"%s\"}", to_string(p_state), to_string(exec));

        switch (p_state)
        {
        case UnknownUser: // the user is still unknown..
            // we start a dialogue for gathering a personalization profile from the user..
            p_state = Talking;
            break;
        case KnownUser: // we now know the user..
            /*
            * Plan management
            */
            if (!exec) // we create a new plan..
                create_new_plan();
            else
                switch (exec->get_solver_state())
                {
                case Finished:
                case Inconsistent:
                    // we make some cleanings before creating a new plan..
                    delete exec;
                    create_new_plan();
                    break;
                case Executing:
                    // we call the executor's tick on a separate thread (notice that this can change the state of the solver)..
                    std::thread(&executor::tick, &exec->get_executor());
                    break;
                default:
                    break;
                }
            break;
        default:
            break;
        }
    }

    void local_task_manager::create_new_plan()
    {
        exec = new ohmni_executor();
        // we create a new planning problem according to the user's profile..
        exec->get_solver().read("");
        if (exec->get_solver_state() != Inconsistent) // we call the solver's solve on a separate thread..
            std::thread(&solver::solve, &exec->get_solver());
    }
} // namespace sir