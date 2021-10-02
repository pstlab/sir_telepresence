#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deliberative_tier");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Deliberative Tier..");

    return 0;
}