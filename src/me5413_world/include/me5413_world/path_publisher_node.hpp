/** path_publisher_node.hpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Declarations for PathPublisherNode class
 */

#ifndef PATH_PUBLISHER_NODE_H_
#define PATH_PUBLISHER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>  // 用于表示一条机器人需要跟踪的路径，由多个点（geometry_msgs/PoseStamped）组成。
#include <nav_msgs/Odometry.h>  // 用于表示机器人的里程计信息，包括位置（pose）和速度（twist）
#include <geometry_msgs/Pose.h>  // geometry_msgs 包的一部分，定义了 Pose 消息类型，用于表示三维空间中的位置（Position）和方向（Orientation）。
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <me5413_world/path_publisherConfig.h>

namespace me5413_world
{

class PathPublisherNode
{
 public:
  PathPublisherNode();
  virtual ~PathPublisherNode(){};  //虚析构

 private:
  void timerCallback(const ros::TimerEvent &);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);  // 订阅中用到的odom回调函数
  void publishGlobalPath();
  void publishLocalPath(const geometry_msgs::Pose &robot_pose, const int n_wp_prev, const int n_wp_post);

  std::vector<geometry_msgs::PoseStamped> createGlobalPath(const double A, const double B, const double t_res);
  int closestWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start);
  int nextWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start);
  double getYawFromOrientation(const geometry_msgs::Quaternion &orientation);
  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose &pose);
  std::pair<double, double> calculatePoseError(const geometry_msgs::Pose &pose_robot, const geometry_msgs::Pose &pose_goal);

  // ROS declaration
  ros::NodeHandle nh_;  // 创建ros节点句柄
  ros::Timer timer_;  // 定时器
  tf2_ros::Buffer tf2_buffer_;  // TF2缓存，用于存储坐标系变换
  tf2_ros::TransformListener tf2_listener_;  // 监听TF坐标系变换
  tf2_ros::TransformBroadcaster tf2_bcaster_;  //坐标变换的广播器
  dynamic_reconfigure::Server<me5413_world::path_publisherConfig> server;  // 动态参数服务器
  dynamic_reconfigure::Server<me5413_world::path_publisherConfig>::CallbackType f;  // 动态参数回调函数类型

  ros::Subscriber sub_robot_odom_;

  ros::Publisher pub_global_path_;
  ros::Publisher pub_local_path_;
  ros::Publisher pub_abs_position_error_;
  ros::Publisher pub_abs_heading_error_;
  ros::Publisher pub_abs_speed_error_;
  ros::Publisher pub_rms_position_error_;
  ros::Publisher pub_rms_heading_error_;
  ros::Publisher pub_rms_speed_error_;

  // Robot pose
  std::string world_frame_;
  std::string robot_frame_;

  geometry_msgs::Pose pose_world_goal_;
  nav_msgs::Odometry odom_world_robot_;

  nav_msgs::Path global_path_msg_;
  nav_msgs::Path local_path_msg_;

  std_msgs::Float32 abs_position_error_;
  std_msgs::Float32 abs_heading_error_;
  std_msgs::Float32 abs_speed_error_;
  std_msgs::Float32 rms_position_error_;
  std_msgs::Float32 rms_heading_error_;
  std_msgs::Float32 rms_speed_error_;

  int current_id_;
  long long num_time_steps_;  // long long 是一种带符号的整数类型，表示可以存储更大的整数范围。(-2^63 ~ 2^63-1)
  double sum_sqr_position_error_;
  double sum_sqr_heading_error_;
  double sum_sqr_speed_error_;
};

} // namespace me5413_world

#endif // PATH_PUBLISHER_NODE_H_
