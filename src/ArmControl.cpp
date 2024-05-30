#include <ArmControl.h>

ArmControl::ArmControl(const std::string &robot_name): name(robot_name)
{
    ros::NodeHandle nd;
    pathSub = nd.subscribe("/myPath", 1, &ArmControl::getPath, this);
    stateSub = nd.subscribe("/" + robot_name  + "/action_topic", 1, &ArmControl::notification_callback,this);
    positionSub = nd.subscribe("/my_gen3_lite/joint_states", 1, &ArmControl::getPosition, this);
    realPath.header.frame_id = "base_link";
    real_Path_pub=nd.advertise<nav_msgs::Path>("/realPath", 1);
    dh_params_list << 0, 0, 243.3 / 1000, 0,
                      M_PI / 2, 0, 10 / 1000, 0 + M_PI / 2,
                      M_PI, 280 / 1000, 0, 0 + M_PI / 2,
                      M_PI / 2, 0, 245 / 1000, 0 + M_PI / 2,
                      M_PI / 2, 0, 57 / 1000, 0,
                      -M_PI / 2, 0, 235 / 1000, 0 - M_PI / 2;
    success = ArmControl::setReferenceFrame();
}

void ArmControl::notification_callback(const kortex_driver::ActionNotification& notif)
{
    last_action_notification_event = notif.action_event;
    last_action_notification_id = notif.handle.identifier;
}

bool ArmControl::wait_for_action_end_or_abort()
{
    while (ros::ok())
    {
        if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
        {
            ROS_INFO("Received ACTION_END notification for action %d", last_action_notification_id.load());
            return true;
        }
        else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
        {
            ROS_INFO("Received ACTION_ABORT notification for action %d", last_action_notification_id.load());
            all_notifs_succeeded = false;
            return false;
        }
        ros::spinOnce();
    }
    return false;
}

bool ArmControl::goHome()
{
    ros::ServiceClient service_client_read_action = nd.serviceClient<kortex_driver::ReadAction>("/" + name + "/base/read_action");
    kortex_driver::ReadAction service_read_action;

    // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

    if (!service_client_read_action.call(service_read_action))
    {
        std::string error_string = "Failed to call ReadAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    // We can now execute the Action that we read 
    ros::ServiceClient service_client_execute_action = nd.serviceClient<kortex_driver::ExecuteAction>("/" + name + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input = service_read_action.response.output;
    
    if (service_client_execute_action.call(service_execute_action))
    {
        ROS_INFO("The Home position action was sent to the robot.");
    }
    else
    {
        std::string error_string = "Failed to call ExecuteAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    return wait_for_action_end_or_abort();
}

bool ArmControl::goPose(const geometry_msgs::Pose &pose, uint32_t id)
{
    kortex_driver::ConstrainedPose my_constrained_pose;
    kortex_driver::CartesianSpeed my_cartesian_speed;

    my_cartesian_speed.translation = 0.1f;
    my_cartesian_speed.orientation = 15.0f;

    my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

    my_constrained_pose.target_pose.x = pose.position.x;
    my_constrained_pose.target_pose.y = pose.position.y;
    my_constrained_pose.target_pose.z = pose.position.z;
    my_constrained_pose.target_pose.theta_x = 90.0;
    my_constrained_pose.target_pose.theta_y = 0.0;
    my_constrained_pose.target_pose.theta_z = 150.0;

    ros::ServiceClient service_client_execute_action = nd.serviceClient<kortex_driver::ExecuteAction>("/" + name + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
    service_execute_action.request.input.name = "pose";
    service_execute_action.request.input.handle.identifier = id;
    service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;

    last_action_notification_event = 0;
    if (service_client_execute_action.call(service_execute_action))
    {
    ROS_INFO("Pose was sent to the robot.");
    }
    else
    {
    std::string error_string = "Failed to call ExecuteAction on pose";
    ROS_ERROR("%s", error_string.c_str());
    return false;
    }

    // Waiting for the pose 1 to end
    wait_for_action_end_or_abort();
    
    return true;
}

bool ArmControl::setReferenceFrame()
{
    ros::ServiceClient service_client_set_cartesian_reference_frame = nd.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + name + "/control_config/set_cartesian_reference_frame");
    kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

    service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
    if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
    {
        std::string error_string = "Failed to call SetCartesianReferenceFrame";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
    else
    {
        ROS_INFO("Set reference to base frame.");
    }

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    return true;
}

bool ArmControl::followPath()
{
    std::vector<double> current_joint_angles = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < myPath.poses.size(); i++)
    {
        // success=goPose(myPath.poses[i].pose, i);
        // if(!success)
        // {
        //     return false;
        // }
        std::vector<double> poses = {myPath.poses[i].pose.position.x, myPath.poses[i].pose.position.y, myPath.poses[i].pose.position.z};
        std::vector<double> target_angles = ArmControl::inverse_kinematics(poses, current_joint_angles);
        std::cout<<"target_angles: "<<target_angles[0]<<", "<<target_angles[1]<<", "<<target_angles[2]<<", "<<target_angles[3]<<", "<<target_angles[4]<<", "<<target_angles[5]<<std::endl;
        current_joint_angles = target_angles;
    }
    return true;
}

void ArmControl::getPath(const nav_msgs::Path::ConstPtr msg)
{
    myPath = *msg;
}

void ArmControl::getPosition(const sensor_msgs::JointState::ConstPtr msg)
{
    positions = msg->position;
    ArmControl::plotRealPath();
}

void ArmControl::plotRealPath()
{
    geometry_msgs::PoseStamped real_poses;
    real_poses.header.frame_id = "base_link";
    realPath.header.stamp = ros::Time::now();
    real_poses.header.stamp=ros::Time::now();
    std::vector<double> tip_pose = get_tip_pose_DH(positions);
    real_poses.pose.position.x = tip_pose[0];
    real_poses.pose.position.y = tip_pose[1];
    real_poses.pose.position.z = tip_pose[2];
    realPath.poses.push_back(real_poses);
    real_Path_pub.publish(realPath);
}

Eigen::Matrix4d ArmControl::gen_T(float theta, float a, float alpha, float d) {
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::cos(alpha)*std::sin(theta), std::sin(theta)*std::sin(alpha), a*std::cos(theta),
         std::sin(theta), std::cos(alpha)*std::cos(theta), -std::cos(theta)*std::sin(alpha), a*std::sin(theta),
         0, std::sin(alpha), std::cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d ArmControl::forward_kinematics(std::vector<double> theta){
    theta[1] += M_PI / 2;
    theta[2] += M_PI / 2;
    theta[3] += M_PI / 2;
    theta[4] += M_PI;
    theta[5] += M_PI / 2;
    Eigen::VectorXd alpha(6);
    alpha << M_PI / 2, M_PI, M_PI / 2, M_PI / 2, M_PI / 2, 0;
    Eigen::VectorXd a(6);
    a << 0, 280, 0, 0, 0, 0;
    Eigen::VectorXd d(6);
    d << 128.3 + 115.0, 30, 20, 140 + 105, 28.5 + 28.5, 105 + 130;
    Eigen::Matrix4d T0_1 = ArmControl::gen_T(theta[0], a[0], alpha[0], d[0]);
    Eigen::Matrix4d T1_2 = ArmControl::gen_T(theta[1], a[1], alpha[1], d[1]);
    Eigen::Matrix4d T2_3 = ArmControl::gen_T(theta[2], a[2], alpha[2], d[2]);
    Eigen::Matrix4d T3_4 = ArmControl::gen_T(theta[3], a[3], alpha[3], d[3]);
    Eigen::Matrix4d T4_5 = ArmControl::gen_T(theta[4], a[4], alpha[4], d[4]);
    Eigen::Matrix4d T5_6 = ArmControl::gen_T(theta[5], a[5], alpha[5], d[5]);
    Eigen::Matrix4d T_rote;
    T_rote << 0, -1, 0, 0,
              1, 0, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Eigen::Matrix4d T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T_rote;
    return T0_6;
}

std::vector<double> ArmControl::get_tip_pose_DH(std::vector<double> theta) {
    Eigen::Matrix4d T0_6 = ArmControl::forward_kinematics(theta);
    std::vector<double> tip_pose = {T0_6(0, 3)/1000, T0_6(1, 3)/1000, T0_6(2, 3)/1000};
    return tip_pose;
}

// 目标函数
double ArmControl::objective_function(std::vector<double> theta, std::vector<double> target_position) {
    Eigen::Vector3d target_position_vec = Eigen::Map<Eigen::Vector3d>(target_position.data(), target_position.size());
    Eigen::Vector3d end_effector_position = forward_kinematics(theta).block<3, 1>(0, 3);
    return (end_effector_position - target_position_vec).norm();
}

double ArmControl::constrain_angle(double angle) {
    return std::fmod(angle + M_PI, 2*M_PI) - M_PI;
}

// 最小化目标函数
std::vector<double> ArmControl::minimize(std::function<double(std::vector<double>)> func, std::vector<double> initial_guess) {
    /// 使用简单的梯度下降法作为示例，实际中可能需要更复杂的优化算法
    double alpha = 0.1;
    std::vector<double> theta = initial_guess;
    for (int i = 0; i < 1000; ++i) {
        std::vector<double> gradient(theta.size());
        for (int j = 0; j < theta.size(); ++j) {
            std::vector<double> theta_plus = theta;
            theta_plus[j] += 0.001;
            std::vector<double> theta_minus = theta;
            theta_minus[j] -= 0.001;
            gradient[j] = (func(theta_plus) - func(theta_minus)) / 0.002;
        }
        for (int j = 0; j < theta.size(); ++j) {
            theta[j] -= alpha * gradient[j];
            theta[j] = constrain_angle(theta[j]);
        }
    }
    return theta;
}

// 逆运动学解算函数
std::vector<double> ArmControl::inverse_kinematics(std::vector<double> target_position, std::vector<double> current_joint_angles) {
    std::function<double(std::vector<double>)> func = [this, target_position](std::vector<double> theta) {
        return objective_function(theta, target_position);
    };
    return minimize(func, current_joint_angles);
}