#pragma once

#include <thread>
#include <atomic>

#include "ros/ros.h"
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <cmath>

#define HOME_ACTION_IDENTIFIER 2

class ArmControl
{
public:
    nav_msgs::Path myPath={};
    bool success = false;

    ArmControl(const std::string &robot_name);
    void notification_callback(const kortex_driver::ActionNotification& notif);
    bool wait_for_action_end_or_abort();
    bool goHome();
    bool goPose(const geometry_msgs::Pose &pose, uint32_t id);
    bool setReferenceFrame();
    bool followPath();
    void getPath(const nav_msgs::Path::ConstPtr msg);
    void getPosition(const sensor_msgs::JointState::ConstPtr msg);
    void plotRealPath();

private:
    std::atomic<int> last_action_notification_event{0};
    std::atomic<int> last_action_notification_id{0};
    std::vector<double> positions;
    nav_msgs::Path realPath;
    bool all_notifs_succeeded = true;
    std::string name;
    ros::NodeHandle nd;
    ros::Subscriber pathSub;
    ros::Subscriber stateSub;
    ros::Subscriber positionSub;
    ros::Publisher real_Path_pub;
    Eigen::Matrix<double, 6, 4> dh_params_list;
    Eigen::Matrix4d gen_T(float theta, float a, float alpha, float d);
    Eigen::Matrix4d forward_kinematics(std::vector<double> theta);
    std::vector<double> get_tip_pose_DH(std::vector<double> theta);
    double objective_function(std::vector<double> theta, std::vector<double> target_position);
    double constrain_angle(double angle);
    std::vector<double> minimize(std::function<double(std::vector<double>)> func, std::vector<double> initial_guess);
    std::vector<double> inverse_kinematics(std::vector<double> target_position, std::vector<double> current_joint_angles);

};









