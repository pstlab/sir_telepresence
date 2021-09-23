#pragma once

#include "executor_listener.h"

namespace sir
{
  class local_task_manager : public ratio::executor_listener
  {
  public:
    local_task_manager(ratio::executor &e);
    ~local_task_manager();

  private:
  };
} // namespace sir
