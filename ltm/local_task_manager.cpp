#include "local_task_manager.h"

namespace sir
{
    local_task_manager::local_task_manager(ratio::executor &e) : ratio::executor_listener(e) {}
    local_task_manager::~local_task_manager() {}
} // namespace sir