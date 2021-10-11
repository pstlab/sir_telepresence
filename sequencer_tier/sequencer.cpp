#include "sequencer.h"

namespace sir
{
    sequencer::sequencer(ros::NodeHandle &handle) : handle(handle), deliberative_state_sub(handle.subscribe("deliberative_state", 100, &sequencer::update_deliberative_state, this)) {}
    sequencer::~sequencer() {}

    void sequencer::update_deliberative_state(const msgs::deliberative_state &msg) {}
} // namespace sir
