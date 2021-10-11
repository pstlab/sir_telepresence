#pragma once

#include "msgs/deliberative_state.h"
#include <ros/ros.h>

namespace sir
{
  class sequencer
  {
  public:
    sequencer(ros::NodeHandle &handle);
    ~sequencer();

  private:
    void update_deliberative_state(const msgs::deliberative_state &msg);

  private:
    ros::NodeHandle &handle;
    ros::Subscriber deliberative_state_sub;
  };
} // namespace sir
