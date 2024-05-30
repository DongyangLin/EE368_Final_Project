#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <ArmControl.h>

int main(int argc, char** argv)
{
    std::string name="my_gen3_lite";
    ros::init(argc, argv, "arm_node");
    ArmControl arm(name);
    if(!arm.success)
    {
        ROS_ERROR("Failed to set reference frame");
        return 1;
    }
    ros::NodeHandle nd;
    ros::ServiceClient service_client_activate_notif = nd.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + name + "/base/activate_publishing_of_action_topic");
    kortex_driver::OnNotificationActionTopic service_activate_notif;
    if (service_client_activate_notif.call(service_activate_notif))
    {
        ROS_INFO("Action notification activated!");
    }
    else 
    {
        std::string error_string = "Action notification publication failed";
        ROS_ERROR("%s", error_string.c_str());
        return 1;
    }

    // wait for the path to be published
    while(arm.myPath.poses.size()==0)
    {
        ros::spinOnce();
    }
    // Setting robot home
    bool success = arm.goHome();

    // Follow the path
    success = arm.followPath();

    return success ? 0 : 1;

}