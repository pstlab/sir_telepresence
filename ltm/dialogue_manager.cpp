#include "dialogue_manager.h"

namespace sir
{
    dialogue_manager::dialogue_manager(local_task_manager &ltm) : ltm(ltm) {}
    dialogue_manager::~dialogue_manager() {}

    void dialogue_manager::gather_profile()
    {
        d_state = Talking;
    }

    void dialogue_manager::start_command(const ratio::atom &atm)
    {
        d_state = Talking;
    }
} // namespace sir
