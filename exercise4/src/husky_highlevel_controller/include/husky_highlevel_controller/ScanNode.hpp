#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <string>

namespace husky_highlevel_controller {

// ScanNode will subscribe to a user-specified topic ("/scan") and 
// have a queue limit specified by the user.
// ScanNode will print out the minimum value of the scans it 
// receives from the laser scanner.
class ScanNode {
public:
    ScanNode(ros::NodeHandle& nodeHandle);
    virtual ~ScanNode();

private:
    // Print out the minimum value from scan. If there is no valid measurement 
    // from scan then throw an error. 
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    ros::NodeHandle& nh_;
    // sub_ subscribes to the laser sensor.
    ros::Subscriber sub_;
    // cmdPub_ publishes velocities to the motor.
    ros::Publisher cmdPub_;
    // markerPub_ publishes the location of the pillar to rviz.
    ros::Publisher markerPub_;

    // User-specified parameters.
    std::string topicName_;
    int queueSize_;
    float pGain_;

};

} // namespace husky_highlevel_controller
