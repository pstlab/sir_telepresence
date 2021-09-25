#include "local_task_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ltm");
    ros::NodeHandle nh;

    ROS_INFO("Starting Local Task Manager..");
    sir::local_task_manager exec(nh);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        exec.tick();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
