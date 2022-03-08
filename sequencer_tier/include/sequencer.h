#pragma once

#include "sequencer_tier/sequencer_state.h"
#include "deliberative_tier/deliberative_state.h"
#include "dialogue_manager/dialogue_state.h"
#include "dialogue_manager/set_reminder.h"
#include "deliberative_tier/task_service.h"
#include "deliberative_tier/task_finished.h"
#include <ros/ros.h>
#include <queue>

namespace sir
{
  class service_call;
  class task_finished_service_call;

  class sequencer
  {
    friend class task_finished_service_call;

  public:
    sequencer(ros::NodeHandle &handle);
    ~sequencer();

    void tick();

  private:
    bool can_start(deliberative_tier::task_service::Request &req, deliberative_tier::task_service::Response &res);
    bool start_task(deliberative_tier::task_service::Request &req, deliberative_tier::task_service::Response &res);
    bool can_end(deliberative_tier::task_service::Request &req, deliberative_tier::task_service::Response &res);
    bool end_task(deliberative_tier::task_service::Request &req, deliberative_tier::task_service::Response &res);

    bool set_reminder(dialogue_manager::set_reminder::Request &req, dialogue_manager::set_reminder::Response &res);

    void updated_deliberative_state(const deliberative_tier::deliberative_state &msg) { deliberative_state[msg.reasoner_id] = msg.deliberative_state; }
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
    ros::ServiceServer can_end_server;
    ros::ServiceServer end_task_server;
    // Connection with the physical exercises manager..
    ros::ServiceClient start_physical_exercise_task;
    // Connection with the dialogue manager..
    ros::ServiceClient start_dialogue_task;
    ros::ServiceClient set_dialogue_parameters;
    ros::ServiceServer set_reminder_server;
    // Connection with the persistence manager..
    ros::ServiceClient load, dump;

    /*
     * The sequencer state
     */
    unsigned int sequencer_state = sequencer_tier::sequencer_state::unconfigured;
    ros::Subscriber deliberative_state_sub;
    std::map<uint64_t, unsigned int> deliberative_state;
    ros::Subscriber dialogue_state_sub;
    unsigned int dialogue_state = dialogue_manager::dialogue_state::idle;
    std::queue<service_call *> pending_calls;
  };
} // namespace sir
