#include "sequencer.h"
#include "msgs/start_task.h"

namespace sir
{
    sequencer::sequencer(ros::NodeHandle &handle) : handle(handle), deliberative_state_sub(handle.subscribe("deliberative_state", 100, &sequencer::update_deliberative_state, this)), dialogue_state_sub(handle.subscribe("dialogue_state", 100, &sequencer::update_dialogue_state, this)), start_dialogue(handle.serviceClient<msgs::start_task>("start_dialogue")) {}
    sequencer::~sequencer() {}

    void sequencer::update_deliberative_state(const msgs::deliberative_state &msg) {}
    void sequencer::update_dialogue_state(const msgs::dialogue_state &msg) {}
} // namespace sir
