#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <array>
#include <cmath>

struct Quaternion {
    double x, y, z, w;
};

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

Quaternion eulerToQuaternion(double alpha, double beta, double theta) {
    Quaternion q;
    alpha = degToRad(alpha);
    beta = degToRad(beta);
    theta = degToRad(theta);

    double cos_alpha = cos(alpha / 2);
    double sin_alpha = sin(alpha / 2);
    double cos_beta = cos(beta / 2);
    double sin_beta = sin(beta / 2);
    double cos_theta = cos(theta / 2);
    double sin_theta = sin(theta / 2);

    q.w = cos_alpha * cos_beta * cos_theta + sin_alpha * sin_beta * sin_theta;
    q.x = sin_alpha * cos_beta * cos_theta - cos_alpha * sin_beta * sin_theta;
    q.y = cos_alpha * sin_beta * cos_theta + sin_alpha * cos_beta * sin_theta;
    q.z = cos_alpha * cos_beta * sin_theta - sin_alpha * sin_beta * cos_theta;

    return q;
}

nav_msgs::Path generatePath()
{
    nav_msgs::Path path;
    path.header.frame_id = "base_link";
    path.header.stamp = ros::Time::now();
    // 定义正四面体的顶点，基于home位置(0.435, 0.194, 0.457)
    std::vector<std::array<double, 3>> vertices = {
        {0.435, 0.194, 0.457},         // 顶点 1 (home)
        {0.435 + 0.1, 0.194, 0.457},   // 顶点 2 (沿X轴正方向)
        {0.435, 0.194 + 0.1, 0.457},   // 顶点 3 (沿Y轴正方向)
        {0.435, 0.194, 0.457 + 0.1}    // 顶点 4 (沿Z轴正方向)
    };

    // 定义正四面体的边
    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {1, 2}, {2, 0},  // 底面三条边
        {0, 3}, {1, 3}, {2, 3}   // 三条从底面到顶点的边
    };

    // 生成插值点
    int num_points_per_edge = 10;
    std::vector<std::array<double, 6>> interpolated_poses;

    for (const auto& edge : edges) {
        const auto& start = vertices[edge.first];
        const auto& end = vertices[edge.second];
        for (int j = 0; j < num_points_per_edge; ++j) {
            double t = static_cast<double>(j) / (num_points_per_edge - 1);
            std::array<double, 6> interp_pose;
            for (int i = 0; i < 3; ++i) {
                interp_pose[i] = start[i] * (1 - t) + end[i] * t;
            }
            // 保持角度不变
            interp_pose[3] = 90.0;
            interp_pose[4] = 0.0;
            interp_pose[5] = 150.0;
            interpolated_poses.push_back(interp_pose);
        }
    }

    for (const auto& pose : interpolated_poses) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "base_link";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = pose[0];
        pose_stamped.pose.position.y = pose[1];
        pose_stamped.pose.position.z = pose[2];
        Quaternion q = eulerToQuaternion(pose[3], pose[4], pose[5]);
        pose_stamped.pose.orientation.x = q.x;
        pose_stamped.pose.orientation.y = q.y;
        pose_stamped.pose.orientation.z = q.z;
        pose_stamped.pose.orientation.w = q.w;
        path.poses.push_back(pose_stamped);
    }
    return path;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gen_path_node");
    ros::NodeHandle nd;
    ros::Publisher pathPub = nd.advertise<nav_msgs::Path>("/myPath", 1);
    nav_msgs::Path path = generatePath();
    while(ros::ok())
    {
        pathPub.publish(path);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}