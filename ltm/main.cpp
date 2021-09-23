#include <ros/ros.h>

int main(int argc, char **argv)
{
    ROS_DEBUG("Starting Local Task Manager..");

    ros::init(argc, argv, "LTM");
    ros::NodeHandle nh;
    return 0;
}
