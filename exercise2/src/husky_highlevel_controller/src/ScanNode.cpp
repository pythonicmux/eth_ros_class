#include <husky_highlevel_controller/ScanNode.hpp>

namespace husky_highlevel_controller {

ScanNode::ScanNode(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
    // Get the topic and queue size from the parameters file.
    if(nh_.getParam(ros::this_node::getNamespace() + ros::this_node::getName() +
                "/topic_name", topicName_) == false || 
            nh_.getParam(ros::this_node::getNamespace() + ros::this_node::getName() + 
                "/queue_size", queueSize_) == false) {
        ROS_ERROR("Could not read parameters.\n");
    }

    // Subscribe to the user-specified topic.
    sub_ = nh_.subscribe(topicName_, queueSize_, &ScanNode::scanCallback, this);
}

ScanNode::~ScanNode() {}

void ScanNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Find the closest measurement, if there is a valid one.
    int minIndex = -1;
    float closest = scan->range_max + 1;
    for(int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] >= scan->range_min && scan->ranges[i] < closest) {
            closest = scan->ranges[i];
            minIndex = i;
        }
    }

    if (minIndex < 0) {
        ROS_ERROR("Could not find a valid measurement from the laser scanner.");
    }

    ROS_INFO_STREAM("Measurement " << minIndex << " had the min distance of " << closest << std::endl);
}

} // namespace husky_highlevel_controller
