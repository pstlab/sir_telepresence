#pragma once

#include "sequencer_tier/sequencer_state.h"
#include "deliberative_tier/deliberative_state.h"
#include "msgs/navigation_state.h"
#include "dialogue_manager/dialogue_state.h"
#include "deliberative_tier/can_start.h"
#include "deliberative_tier/start_task.h"
#include "deliberative_tier/task_finished.h"
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
    bool can_start(deliberative_tier::can_start::Request &req, deliberative_tier::can_start::Response &res);
    bool start_task(deliberative_tier::start_task::Request &req, deliberative_tier::start_task::Response &res);

    bool dialogue_task_finished(deliberative_tier::task_finished::Request &req, deliberative_tier::task_finished::Response &res);

    void updated_deliberative_state(const deliberative_tier::deliberative_state &msg) { deliberative_state[msg.reasoner_id] = msg.deliberative_state; }
    void updated_navigation_state(const msgs::navigation_state &msg) { navigation_state = msg.navigation_state; }
    void updated_dialogue_state(const dialogue_manager::dialogue_state &msg) { dialogue_state = msg.dialogue_state; }

    void set_state(const unsigned int &state);

  private:
    ros::NodeHandle &handle;
    ros::Publisher notify_state;
    // Connection with the deliberative tier..
    ros::ServiceClient create_reasoner;
    ros::ServiceClient destroy_reasoner;
    ros::ServiceClient new_requirement;
    ros::ServiceClient task_finished;
    ros::ServiceServer can_start_server;
    ros::ServiceServer start_task_server;
    // Connection with the physical exercises manager..
    ros::ServiceClient start_physical_exercise_task;
    // Connection with the dialogue manager..
    ros::ServiceClient start_dialogue_task;
    ros::ServiceServer dialogue_task_finished_server;

    /*
     * The sequencer state
     */
    unsigned int sequencer_state = sequencer_tier::sequencer_state::unconfigured;
    ros::Subscriber deliberative_state_sub;
    std::map<uint64_t, unsigned int> deliberative_state;
    ros::Subscriber navigation_state_sub;
    unsigned int navigation_state = msgs::navigation_state::idle;
    ros::Subscriber dialogue_state_sub;
    unsigned int dialogue_state = dialogue_manager::dialogue_state::idle;
  };
} // namespace sir
