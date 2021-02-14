#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

ros::Publisher powerPub;

bool toggleRobotPower(std_srvs::SetBoolRequest& request, 
        std_srvs::SetBoolResponse& response) {
    std_msgs::Bool status;
    status.data = request.data;
    powerPub.publish(status);
    response.success = true;

    if(request.data) {
        response.message = "Robot is now on\n";
    } else {
        response.message = "Robot is now off\n";
    }

    return true;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "husky_power_server");
    ros::NodeHandle nodeHandle;

    powerPub = nodeHandle.advertise<std_msgs::Bool>("/robot_power", 1);
    ros::ServiceServer service = nodeHandle.advertiseService("toggle_power", toggleRobotPower);

    ros::spin();
    return 0;
}
