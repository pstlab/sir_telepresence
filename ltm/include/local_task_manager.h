#pragma once

#include <ros/ros.h>

namespace sir
{
  class ohmni_executor;

  class local_task_manager
  {
  public:
    local_task_manager(ros::NodeHandle &handle);
    ~local_task_manager();

    void tick();

  private:
    void create_new_plan();

  private:
    ros::NodeHandle &handle;

    /*
    * Local Task Manager state
    */
    ohmni_executor *exec;
  };
} // namespace sir
