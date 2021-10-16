#include "sequencer.h"

using namespace sir;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequencer_tier");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Sequencer Tier..");

    sequencer seq(nh);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        seq.tick();
        loop_rate.sleep();
    }

    return 0;
}