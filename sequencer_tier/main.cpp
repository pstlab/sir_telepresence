#include "sequencer.h"

using namespace sir;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequencer_tier");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Sequencer Tier..");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

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