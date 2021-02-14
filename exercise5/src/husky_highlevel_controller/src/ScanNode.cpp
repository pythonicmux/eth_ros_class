#include <husky_highlevel_controller/ScanNode.hpp>

namespace husky_highlevel_controller {

ScanNode::ScanNode(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
    robotOn_ = true;
    
    // Get the topic and queue size from the parameters file.
    if(nh_.getParam(ros::this_node::getNamespace() + ros::this_node::getName() +
                "/topic_name", topicName_) == false || 
            nh_.getParam(ros::this_node::getNamespace() + ros::this_node::getName() + 
                "/queue_size", queueSize_) == false) {
        ROS_ERROR("Could not read parameters.\n");
    }
    
    // Get the p-controller gain
    if(nh_.getParam(ros::this_node::getNamespace() + ros::this_node::getName() +
                "/p_gain", pGain_) == false) {
        ROS_ERROR("Could not read p-controller parameters.\n");
    }
    
    // Subscribe to robot turn-on/off commands.
    onSub_ = nh_.subscribe("/robot_power", queueSize_, &ScanNode::toggleRobotOnCallback, this);

    // Subscribe to the user-specified topic.
    sub_ = nh_.subscribe(topicName_, queueSize_, &ScanNode::scanCallback, this);

    // Publish velocities to drive the robot.
    cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    // Publish the location of the pillar.
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

ScanNode::~ScanNode() {}

void ScanNode::toggleRobotOnCallback(std_msgs::Bool isOn) {
    if(isOn.data == false) ROS_INFO_STREAM("Robot is now off.\n");
    else ROS_INFO_STREAM("Robot is now on.\n");
    robotOn_ = isOn.data;
}

void ScanNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // If the robot is off then don't do drive the robot.
    if(!robotOn_) {
        return;
    }

    // Find the closest measurement, if there is a valid one.
    int minIndex = -1;
    float closest = scan->range_max + 1;
    for(int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] >= scan->range_min && scan->ranges[i] < closest) {
            closest = scan->ranges[i];
            minIndex = i;
        }
    }

    // If there's no valid measurements then turn to look for the pillar.
    if (minIndex < 0) {
        ROS_ERROR("Could not find a valid measurement from the laser scanner.");
        geometry_msgs::Twist newVel;
        newVel.angular.z = 0.5;
        cmdPub_.publish(newVel);
        return;
    }

    // Estimate the position of the pillar relative to the robot.
    // The minimum distance displayed by the scanner is the hypotenuse of a triangle.
    // The angle is given in the measurement via a start angle and increment per measurement.
    // The angle is in radians.
    float angle = minIndex * scan->angle_increment + scan->angle_min;
    // We have the hypotenuse and angle, so we find x and y distances of the pillar 
    // from the robot with sine and cosine (they're the legs of the triangle).
    float y = -1.0 * sin(angle) * closest;
    float x = cos(angle) * closest;

    // Display the estimated location of the pillar.
    visualization_msgs::Marker estimatedPillar;
    // /base_link sets the origin of the coordinate plane at the center of mass of the robot. 
    estimatedPillar.header.frame_id = "/base_link";
    estimatedPillar.header.stamp = ros::Time();
    estimatedPillar.type = visualization_msgs::Marker::SPHERE;
    estimatedPillar.action = visualization_msgs::Marker::ADD;

    // Because the origin is at the robot, this marker's position is 
    // specified relative to the robot.
    estimatedPillar.pose.position.x = x;
    estimatedPillar.pose.position.y = y;
    estimatedPillar.pose.position.z = 0;

    estimatedPillar.pose.orientation.x = 0.0;
    estimatedPillar.pose.orientation.y = 0.0;
    estimatedPillar.pose.orientation.z = 0.0;
    estimatedPillar.pose.orientation.w = 1.0;

    estimatedPillar.scale.x = 1.0;
    estimatedPillar.scale.y = 1.0;
    estimatedPillar.scale.z = 1.0;

    estimatedPillar.color.r = 0.0f;
    estimatedPillar.color.g = 1.0f;
    estimatedPillar.color.b = 0.0f;
    estimatedPillar.color.a = 1.0;

    estimatedPillar.lifetime = ros::Duration();

    markerPub_.publish(estimatedPillar);

    // P controller: turn/drive forward at a speed proportional to the 
    // error- the gain is user-specified. 
    // Forward velocity error is proportional to the straight-line distance
    // from the robot to the pillar, in m/s.
    float forwardError = closest * pGain_;
    // Turn velocity error is proportional to the angle that the robot is off by,
    // in radians/second.  
    float turnError = -1.0 * angle * pGain_;

    // Send the velocity to the motors.
    geometry_msgs::Twist newVel;
    newVel.linear.x = forwardError;
    newVel.angular.z = turnError;
    
    cmdPub_.publish(newVel);
}

} // namespace husky_highlevel_controller
