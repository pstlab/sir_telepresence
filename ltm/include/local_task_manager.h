#pragma once

#include <ros/ros.h>

namespace sir
{
  class ohmni_executor;

  enum profile_state
  {
    UnknownUser,
    Talking,
    KnownUser
  };

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
    profile_state p_state = UnknownUser;
  };

  inline const char *to_string(const profile_state &p_state)
  {
    switch (p_state)
    {
    case UnknownUser:
      return "unknown";
    case Talking:
      return "talking";
    case KnownUser:
      return "known";
    default:
      return "-";
    }
  }
} // namespace sir
