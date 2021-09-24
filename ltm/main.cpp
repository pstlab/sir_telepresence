#include "local_task_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ROS_DEBUG("Starting Local Task Manager..");

    ros::init(argc, argv, "local_task_manager");
    ros::NodeHandle nh;

    sir::local_task_manager exec(nh);

    return 0;
}
