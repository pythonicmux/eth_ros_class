#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber laserPub;
ros::ServiceClient client;
int turnOffDistance;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Find the closest measurement, if there is a valid one.
    int minIndex = -1;
    float closest = scan->range_max + 1;
    for(int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] >= scan->range_min && scan->ranges[i] < closest) {
            closest = scan->ranges[i];
            minIndex = i;
        }
    }

    std_srvs::SetBoolRequest request;
    std_srvs::SetBoolResponse response;

    // If there's no valid measurements then turn on the robot and 
    // let it do stuff.
    if (minIndex < 0) {
        ROS_ERROR("Could not find a valid measurement from the laser scanner.");
        request.data = true;
        client.call(request, response);
    // Turn off the robot if it gets too close to something.
    } else if (closest < turnOffDistance) {
        ROS_INFO_STREAM("Laser scan saw pillar " << closest << " units (?) away, turning off...\n");
        request.data = false;
    // Otherwise, turn it on. Maybe a false reading turned it off by accident.
    } else {
        ROS_INFO_STREAM("Laser scan saw pillar " << closest << " units (?) away\n");
        request.data = true;
    }
    
    client.call(request, response);
    if(response.success == false) {
        ROS_ERROR("Attempting to call the toggle_power service failed.");
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "husky_collision_avoidance");
    ros::NodeHandle nodeHandle;

    laserPub = nodeHandle.subscribe("/scan", 10, scanCallback);
    client = nodeHandle.serviceClient<std_srvs::SetBool>("toggle_power");

    if(nodeHandle.getParam(ros::this_node::getNamespace() + ros::this_node::getName() + 
                "/turnOffDistance", turnOffDistance) == false) {
        ROS_ERROR("Could not read parameters for collision avoidance.");
    }

    ros::spin();

    return 0;
}
