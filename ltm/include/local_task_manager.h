#pragma once

#include "ohmni_executor.h"
#include "dialogue_manager.h"
#include <ros/ros.h>

namespace sir
{
  class ohmni_executor;

  enum system_state
  {
    Unconfigured,
    GatheringProfile,
    ProfileGathered,
    Mapping,
    Configured
  };

  class local_task_manager
  {
  public:
    local_task_manager(ros::NodeHandle &handle);
    ~local_task_manager();

    ros::NodeHandle &get_handle() { return handle; }
    ohmni_executor *get_executor() { return exec; }

    void tick();

  private:
    void create_new_plan();

  private:
    ros::NodeHandle &handle;

    /*
    * Local Task Manager state
    */
    ohmni_executor *exec;
    system_state s_state = Unconfigured;
    dialogue_manager d_manager;
  };

  inline const char *to_string(const system_state &s_state)
  {
    switch (s_state)
    {
    case Unconfigured:
      return "\"Unconfigured\"";
    case GatheringProfile:
      return "\"GatheringProfile\"";
    case ProfileGathered:
      return "\"ProfileGathered\"";
    case Mapping:
      return "\"Mapping\"";
    case Configured:
      return "\"Configured\"";
    default:
      return "\"-\"";
    }
  }
} // namespace sir
