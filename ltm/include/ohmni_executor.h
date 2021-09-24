#pragma once

#include "executor_listener.h"

namespace sir
{
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
    ohmni_executor();
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

  private:
    ratio::solver slv;
    ratio::executor exec;
    solver_state state = Idle;
  };
} // namespace sir
