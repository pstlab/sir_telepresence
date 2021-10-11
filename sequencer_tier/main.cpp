#include "sequencer.h"

using namespace sir;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequencer_tier");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Sequencer Tier..");

    sequencer seq(nh);

    ros::spin();
    return 0;
}