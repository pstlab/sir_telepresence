#pragma once

#include "executor_listener.h"
#include <ros/ros.h>

namespace sir
{
  class local_task_manager : public ratio::core_listener, public ratio::executor_listener
  {
  public:
    local_task_manager(ros::NodeHandle &handle);
    ~local_task_manager();

    void started_solving() override;
    void solution_found() override;
    void inconsistent_problem() override;

    void tick(const smt::rational time) override;

    void starting(const std::unordered_set<ratio::atom *> &) override;
    void start(const std::unordered_set<ratio::atom *> &) override;

    void ending(const std::unordered_set<ratio::atom *> &) override;
    void end(const std::unordered_set<ratio::atom *> &) override;

  private:
    ros::NodeHandle &handle;
    ratio::solver slv;
    ratio::executor exec;
  };
} // namespace sir
