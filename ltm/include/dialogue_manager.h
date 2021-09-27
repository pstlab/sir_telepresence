#pragma once

#include "atom.h"
#include "ltm/dialogue_finished.h"
#include <ros/ros.h>

namespace sir
{
  class local_task_manager;

  enum dialogue_state
  {
    Silent,
    Talking
  };

  class dialogue_manager
  {
  public:
    dialogue_manager(local_task_manager &ltm);
    ~dialogue_manager();

    const dialogue_state &get_dialogue_state() const { return d_state; }

    void gather_profile();
    void start_dialogue(const ratio::atom &atm);

  private:
    bool dialogue_finished(ltm::dialogue_finished::Request &req, ltm::dialogue_finished::Response &res);

  private:
    local_task_manager &ltm;
    dialogue_state d_state = Silent;
    const ratio::atom *current_dialogue = nullptr;
  };

  inline const char *to_string(const dialogue_manager &d_manager)
  {
    switch (d_manager.get_dialogue_state())
    {
    case Silent:
      return "\"Silent\"";
    case Talking:
      return "\"Talking\"";
    default:
      return "\"-\"";
    }
  }
} // namespace sir
