#pragma once

#include "executor_listener.h"
#include "ltm/task_finished.h"

namespace sir
{
  class local_task_manager;

  enum solver_state
  {
    Idle,
    Solving,
    Executing,
    Finished,
    Inconsistent
  };

  class ohmni_executor : public ratio::core_listener, public ratio::executor_listener
  {
  public:
    ohmni_executor(local_task_manager &ltm);
    ~ohmni_executor();

    ratio::solver &get_solver() { return slv; }
    ratio::executor &get_executor() { return exec; }

    const solver_state &get_solver_state() const { return state; }

    void started_solving() override;
    void solution_found() override;
    void inconsistent_problem() override;

    void tick(const smt::rational &time) override;

    void starting(const std::unordered_set<ratio::atom *> &) override;
    void start(const std::unordered_set<ratio::atom *> &) override;

    void ending(const std::unordered_set<ratio::atom *> &) override;
    void end(const std::unordered_set<ratio::atom *> &) override;

    friend const char *to_string(const ohmni_executor *exec);

  private:
    bool task_finished(ltm::task_finished::Request &req, ltm::task_finished::Response &res);

  private:
    local_task_manager &ltm;
    ratio::solver slv;
    ratio::executor exec;
    solver_state state = Idle;
    std::unordered_map<smt::var, ratio::atom *> current_tasks;
  };

  inline const char *to_string(const ohmni_executor *exec)
  {
    if (exec)
      switch (exec->get_solver_state())
      {
      case Idle:
        return "\"Idle\"";
      case Solving:
        return "\"Solving\"";
      case Executing:
        return ("\"Executing(" + std::to_string(exec->current_tasks.size()) + ")\"").c_str();
      case Finished:
        return "\"Finished\"";
      case Inconsistent:
        return "\"Inconsistent\"";
      default:
        return "\"-\"";
      }
    else
      return "null";
  }
} // namespace sir
