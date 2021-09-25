#include "dialogue_manager.h"
#include "local_task_manager.h"
#include "predicate.h"

namespace sir
{
    dialogue_manager::dialogue_manager(local_task_manager &ltm) : ltm(ltm) {}
    dialogue_manager::~dialogue_manager() {}

    void dialogue_manager::gather_profile()
    {
        ROS_INFO("Starting profile gathering phase..");
        d_state = Talking;
        //ltm.get_handle().advertiseService("profile_gathered", []()
        //                                  { return true; });
    }

    void dialogue_manager::start_dialogue(const ratio::atom &atm)
    {
        ROS_ASSERT(current_dialogue == nullptr);
        ROS_INFO("Starting dialogue %s..", atm.get_type().get_name());
        d_state = Talking;
        current_dialogue = &atm;
    }
} // namespace sir
