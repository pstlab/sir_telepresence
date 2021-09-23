#pragma once

#include "executor_listener.h"

namespace sir
{
  class local_task_manager : public ratio::executor_listener
  {
  public:
    local_task_manager();
    ~local_task_manager();

    void starting(const std::unordered_set<ratio::atom *> &) override;
    void start(const std::unordered_set<ratio::atom *> &) override;

    void ending(const std::unordered_set<ratio::atom *> &) override;
    void end(const std::unordered_set<ratio::atom *> &) override;

  private:
    ratio::solver slv;
    ratio::executor exec;
  };
} // namespace sir
