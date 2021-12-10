#include "service_call.h"
#include "deliberative_tier/task_finished.h"
#include "sequencer.h"

namespace sir
{
    bool task_finished_service_call::call() const
    {
        deliberative_tier::task_finished dtf_srv;
        dtf_srv.request.reasoner_id = reasoner_id;
        dtf_srv.request.task_id = task_id;
        dtf_srv.request.task_name = task_name;
        dtf_srv.request.success = success;
        return seq.task_finished.call(dtf_srv);
    }
} // namespace sir
