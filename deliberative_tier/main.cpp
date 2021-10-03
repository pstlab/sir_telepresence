#include "deliberative_manager.h"

using namespace sir;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deliberative_tier");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Deliberative Tier..");

    deliberative_manager dm(nh);

    ros::spin();
    return 0;
}