#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace sir
{
  class sequencer;

  class service_call
  {
  public:
    service_call(sequencer &seq) : seq(seq) {}
    virtual ~service_call() {}

    virtual bool call() const = 0;

  protected:
    sequencer &seq;
  };

  class task_finished_service_call : public service_call
  {
  public:
    task_finished_service_call(sequencer &seq, const uint64_t &reasoner_id, const uint64_t &task_id, const std::string &task_name, const std::vector<std::string> &par_names, const std::vector<std::string> &par_values, const bool &success) : service_call(seq), reasoner_id(reasoner_id), task_id(task_id), par_names(par_names), par_values(par_values), success(success) {}
    ~task_finished_service_call() {}

  private:
    bool call() const override;

  private:
    uint64_t reasoner_id;
    uint64_t task_id;
    std::string task_name;
    std::vector<std::string> par_names;
    std::vector<std::string> par_values;
    bool success;
  };
} // namespace sir
