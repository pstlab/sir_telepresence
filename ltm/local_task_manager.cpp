#include "local_task_manager.h"
#include <thread>

using namespace ratio;

namespace sir
{
    local_task_manager::local_task_manager(ros::NodeHandle &handle) : handle(handle), d_manager(*this) {}
    local_task_manager::~local_task_manager() {}

    void local_task_manager::tick()
    {
        ROS_INFO("{\"System\": %s, \"Dialogue\": %s, \"Solver\": %s}", to_string(s_state), to_string(d_manager), to_string(exec));

        switch (s_state)
        {
        case Unconfigured: // the user is still unknown..
            // we start a dialogue for gathering a personalization profile from the user..
            d_manager.gather_profile();
            s_state = GatheringProfile;
            break;
        case GatheringProfile: // we are gathering the user's profile..
            break;
        case ProfileGathered: // we now know the user..
            // we start the mapping phase..
            s_state = Mapping;
            break;
        case Mapping: // we are mapping the environment..
            break;
        case Configured: // the system is now configured..
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
                case Solving:
                    // we should recover these steps..
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
        exec = new ohmni_executor(*this);
        // we create a new planning problem according to the user's profile..
        exec->get_solver().read("");
        if (exec->get_solver_state() != Inconsistent) // we call the solver's solve on a separate thread..
            std::thread(&solver::solve, &exec->get_solver());
    }
} // namespace sir