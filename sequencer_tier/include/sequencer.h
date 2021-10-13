#pragma once

#include "msgs/deliberative_state.h"
#include "msgs/dialogue_state.h"
#include <ros/ros.h>

namespace sir
{
  class sequencer
  {
  public:
    sequencer(ros::NodeHandle &handle);
    ~sequencer();

  private:
    void updated_deliberative_state(const msgs::deliberative_state &msg);
    void updated_dialogue_state(const msgs::dialogue_state &msg);

  private:
    ros::NodeHandle &handle;
    ros::Subscriber deliberative_state_sub;
    ros::Subscriber dialogue_state_sub;
    ros::ServiceClient start_dialogue;
  };
} // namespace sir
