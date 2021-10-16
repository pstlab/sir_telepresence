#pragma once

#include "msgs/deliberative_state.h"
#include "msgs/dialogue_state.h"
#include "msgs/system_state.h"
#include <ros/ros.h>

namespace sir
{
  class sequencer
  {
  public:
    sequencer(ros::NodeHandle &handle);
    ~sequencer();

    void tick();

  private:
    void updated_system_state(const msgs::system_state &msg) { system_state = msg.system_state; }
    void updated_deliberative_state(const msgs::deliberative_state &msg) { deliberative_state = msg.reasoner_state; }
    void updated_dialogue_state(const msgs::dialogue_state &msg) { dialogue_state = msg.dialogue_state; }

    void set_state(const unsigned int &state);

  private:
    ros::NodeHandle &handle;
    ros::Publisher notify_state;

    /*
     * The sequencer state
     */
    ros::Subscriber system_state_sub;
    unsigned int system_state = msgs::system_state::unconfigured;
    ros::Subscriber deliberative_state_sub;
    unsigned int deliberative_state = msgs::deliberative_state::idle;
    ros::Subscriber dialogue_state_sub;
    unsigned int dialogue_state = msgs::dialogue_state::idle;
    ros::ServiceClient start_dialogue_service;
  };
} // namespace sir
