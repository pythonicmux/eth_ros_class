#include <ros/ros.h>
#include <husky_highlevel_controller/ScanNode.hpp>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "husky_highlevel_controller_node");
    ros::NodeHandle nodeHandle;

    husky_highlevel_controller::ScanNode sn(nodeHandle);

    ros::spin();
    return 0;
}
