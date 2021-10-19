#pragma once

#include "executor_listener.h"

namespace sir
{
  class deliberative_manager;

  enum executor_state
  {
    Idle,
    Reasoning,
    Executing,
    Finished,
    Inconsistent
  };

  class deliberative_executor : public ratio::executor_listener
  {
  public:
    deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id);
    ~deliberative_executor();

    ratio::solver &get_solver() { return slv; }
    ratio::executor &get_executor() { return exec; }

    void finish_task(const smt::var &id, const bool &success = true);

  private:
    void tick(const smt::rational &time) override;

    void starting(const std::unordered_set<ratio::atom *> &) override;
    void start(const std::unordered_set<ratio::atom *> &) override;

    void ending(const std::unordered_set<ratio::atom *> &) override;
    void end(const std::unordered_set<ratio::atom *> &) override;

    void set_state(const executor_state &state);

    struct task
    {
      uint64_t task_id;
      std::string task_name;
      std::vector<std::string> par_names;
      std::vector<std::string> par_values;
    };

    task to_task(const ratio::atom &atm) const;

    class deliberative_core_listener : public ratio::core_listener
    {
    public:
      deliberative_core_listener(deliberative_executor &exec);
      ~deliberative_core_listener();

    private:
      void started_solving() override;
      void solution_found() override;
      void inconsistent_problem() override;

    private:
      deliberative_executor &exec;
    };

  private:
    deliberative_manager &d_mngr;
    uint64_t reasoner_id;
    ratio::solver slv;
    ratio::executor exec;
    deliberative_core_listener dcl;
    executor_state state = Idle;
    std::unordered_map<smt::var, ratio::atom *> current_tasks;
  };
} // namespace sir
