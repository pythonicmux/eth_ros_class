#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
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
    ros::Subscriber sub_;
    std::string topicName_;
    int queueSize_;

};

} // namespace husky_highlevel_controller
