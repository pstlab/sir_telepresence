#include "local_task_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ROS_DEBUG("Starting Local Task Manager..");

    ros::init(argc, argv, "LTM");
    ros::NodeHandle nh;

    sir::local_task_manager ltm(nh);

    return 0;
}
