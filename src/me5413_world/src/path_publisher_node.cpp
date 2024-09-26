/** path_publisher_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for publishing short term paths
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_publisher_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double TRACK_A_AXIS;
double TRACK_B_AXIS;
double TRACK_WP_NUM;
double LOCAL_PREV_WP_NUM;
double LOCAL_NEXT_WP_NUM;
bool PARAMS_UPDATED = false;

// 动态参数回调函数，用于处理来自 dynamic_reconfigure 的参数更新
// config：包含动态参数的配置对象
// level：指示参数更新的级别（当前未使用）
// 可以通过 rqt_reconfigure 图形界面动态调整参数，实时观察表现
void dynamicParamCallback(const me5413_world::path_publisherConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;  // 更新机器人路径跟踪的目标速度
  // Global Path Settings
  TRACK_A_AXIS = config.track_A_axis;  // 更新路径生成参数（椭圆路径的 A 轴长度）
  TRACK_B_AXIS = config.track_B_axis;   // 更新路径生成参数（椭圆路径的 B 轴长度）
  TRACK_WP_NUM = config.track_wp_num;   // 更新路径生成参数（路径点的数量）
  LOCAL_PREV_WP_NUM = config.local_prev_wp_num;  // 更新局部路径生成参数（局部路径中要包括的之前的路径点数量）
  LOCAL_NEXT_WP_NUM = config.local_next_wp_num;  // 更新局部路径生成参数（局部路径中要包括的之后的路径点数量）
  PARAMS_UPDATED = true;  // 标记参数已经更新，这样定时器回调函数会根据新参数重新生成路径
}  

// 构造函数初始化实现
PathPublisherNode::PathPublisherNode() : tf2_listener_(tf2_buffer_)
{
  // 通过bind函数绑定成一个对象，_1, _2代表占位符号，即传入dynamicParamCallback的参数
  // c++11中优先使用std::bind，boost::bind和std::bind功能类似
  // f = std::bind(&dynamicParamCallback, std::placeholders::_1, std::placeholders::_2);
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);  

   // 创建定时器，周期性（0.1s）调用timerCallback函数
  this->timer_ = nh_.createTimer(ros::Duration(0.1), &PathPublisherNode::timerCallback, this);
  
  // 订阅Gazebo仿真中的机器人位姿信息，然后发布坐标变换
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathPublisherNode::robotOdomCallback, this);
  
  // 设置多个话题的发布器，用于发布全局路径、局部路径、以及各种误差信息
  this->pub_global_path_ = nh_.advertise<nav_msgs::Path>("/me5413_world/planning/global_path", 1);
  this->pub_local_path_ = nh_.advertise<nav_msgs::Path>("/me5413_world/planning/local_path", 1);
  this->pub_abs_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/abs_position_error", 1);
  this->pub_abs_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/abs_heading_error", 1);
  this->pub_abs_speed_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/abs_speed_error", 1);
  this->pub_rms_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/rms_position_error", 1);
  this->pub_rms_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/rms_heading_error", 1);
  this->pub_rms_speed_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/rms_speed_error", 1);

  // 初始化坐标系
  this->robot_frame_ = "base_link";  // 机器人底盘坐标系
  this->world_frame_ = "world";  // 世界坐标系

  this->global_path_msg_.header.frame_id = this->world_frame_;
  this->local_path_msg_.header.frame_id = this->world_frame_;
  this->global_path_msg_.poses = createGlobalPath(TRACK_A_AXIS, TRACK_B_AXIS, 1.0/TRACK_WP_NUM);

  // 初始化各类误差为零
  this->abs_position_error_.data = 0.0;  
  this->abs_heading_error_.data = 0.0;
  this->rms_position_error_.data = 0.0;
  this->rms_heading_error_.data = 0.0;

  this->current_id_ = 0;  // 当前路径点索引
  this->num_time_steps_ = 1;  // 时间步计数
  this->sum_sqr_position_error_ = 0.0;  // 位置误差平方和
  this->sum_sqr_heading_error_ = 0.0;  // 朝向误差平方和
}

// 定时器回调函数实现，用于周期性发布路径和误差
void PathPublisherNode::timerCallback(const ros::TimerEvent &)
{
  // Create and Publish Paths
  if (PARAMS_UPDATED)  // 如果参数更新，则重新生成全局路径； 生成一次之后就设置为false
  {
    this->global_path_msg_.poses = createGlobalPath(TRACK_A_AXIS, TRACK_B_AXIS, 1.0/TRACK_WP_NUM);
    this->current_id_ = 0;
    PARAMS_UPDATED = false;
  }

  // 发布全局和局部路径
  publishGlobalPath(); // 函数内发布了全局路径
  publishLocalPath(this->odom_world_robot_.pose.pose, LOCAL_PREV_WP_NUM, LOCAL_NEXT_WP_NUM);

  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> abs_errors = calculatePoseError(this->odom_world_robot_.pose.pose, this->pose_world_goal_);
  this->abs_position_error_.data = abs_errors.first;  // 距离误差
  this->abs_heading_error_.data = abs_errors.second;  // 朝向误差
  tf2::Vector3 velocity;
  // 将geometry_msgs::Vector3的数据转成tf2::Vector3
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, velocity);
  // length() 函数用于计算三维向量（线速度）的 模（magnitude） 或者 长度，也就是这个向量的大小
  this->abs_speed_error_.data = velocity.length() - SPEED_TARGET;

  // Calculate average errors
  this->sum_sqr_position_error_ += std::pow(abs_errors.first, 2);
  this->sum_sqr_heading_error_ += std::pow(abs_errors.second, 2);
  this->sum_sqr_speed_error_ += std::pow(this->abs_speed_error_.data, 2);
  this->rms_position_error_.data = std::sqrt(sum_sqr_position_error_/num_time_steps_);
  this->rms_heading_error_.data = std::sqrt(sum_sqr_heading_error_/num_time_steps_);
  this->rms_speed_error_.data = std::sqrt(sum_sqr_speed_error_/num_time_steps_);

  // Publish errors
  this->pub_abs_position_error_.publish(this->abs_position_error_);
  this->pub_abs_heading_error_.publish(this->abs_heading_error_);
  this->pub_abs_speed_error_.publish(this->abs_speed_error_);
  this->pub_rms_position_error_.publish(this->rms_position_error_);
  this->pub_rms_heading_error_.publish(this->rms_heading_error_);
  this->pub_rms_speed_error_.publish(this->rms_speed_error_);

  // Count
  this->num_time_steps_++;

  return;
}

// odom信息订阅回调函数
void PathPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  // 把机器人world坐标系下的位置和朝向信息由geometry_msgs::Pose转为tf2::Transform格式
  const tf2::Transform T_world_robot = convertPoseToTransform(this->odom_world_robot_.pose.pose);  // 转换为TF变换
  // 从机器人坐标系到世界坐标系的变换
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  // geometry_msgs::TransformStamped 是 ROS 中定义的一种标准消息类型，位于 geometry_msgs 包中，专门用于描述坐标系之间的 变换（包括平移和平面旋转），并附带了时间戳和帧信息。
  // 它常用于 ROS 的 TF 变换系统，用来发布一个坐标系相对于另一个坐标系的变换。
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  // tf2::toMsg转成geometry_msgs数据类型
  transformStamped.transform.translation = tf2::toMsg(T_robot_world.getOrigin());
  // transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = tf2::toMsg(T_robot_world.getRotation());
  this->tf2_bcaster_.sendTransform(transformStamped);  // 发布坐标变化

  return;
}

std::vector<geometry_msgs::PoseStamped> PathPublisherNode::createGlobalPath(const double A, const double B, const double t_res)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  const double t_increament = t_res * 2 * M_PI;

  // Calculate the positions
  for (double t = 0.0; t <= 2 * M_PI; t += t_increament)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = A * std::sin(t);
    pose.pose.position.y = B * std::sin(t) * std::cos(t);
    poses.push_back(pose);
  }

  // Calcuate the orientations
  tf2::Quaternion q;
  for (int i = 0; i < poses.size(); i++)
  {
    const double x_d = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double y_d = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    const double yaw = std::atan2(y_d, x_d);
    // setRPY就是roll,pitch,yaw转四元素
    // getRPY就是四元素转roll，pitch，yaw
    q.setRPY(0.0, 0.0, yaw); // 构造相应的四元数
    q.normalize();
    // 将 tf2::Quaternion 转换为 geometry_msgs::Quaternion 用于 ROS 消息传递
    poses[i].pose.orientation = tf2::toMsg(q);
  }
  poses.back().pose.orientation = tf2::toMsg(q);  // 因为最后一个没法算朝向yaw，所以设置为和前一个一样

  return poses;
}

void PathPublisherNode::publishGlobalPath()
{
  // Update the message
  this->global_path_msg_.header.stamp = ros::Time::now();
  this->pub_global_path_.publish(this->global_path_msg_);
}

void PathPublisherNode::publishLocalPath(const geometry_msgs::Pose &robot_pose, const int n_wp_prev, const int n_wp_post)
{
  int id_next = nextWaypoint(robot_pose, this->global_path_msg_, this->current_id_);
  if (this->global_path_msg_.poses.empty())
  { // 判断全局路径是否发布
    ROS_WARN("Global Path not published yet, waiting");
  }
  else if (id_next >= this->global_path_msg_.poses.size() - 1)
  { // 判断是否到达终点
    ROS_WARN("Robot has reached the end of the track, please restart");
  }
  else
  {
    this->current_id_ = std::max(this->current_id_, id_next - 1);
    int id_start = std::max(id_next - n_wp_prev, 0);
    int id_end = std::min(id_next + n_wp_post, int(this->global_path_msg_.poses.size() - 1));

    std::vector<geometry_msgs::PoseStamped>::const_iterator start = this->global_path_msg_.poses.begin() + id_start;
    std::vector<geometry_msgs::PoseStamped>::const_iterator end = this->global_path_msg_.poses.begin() + id_end;

    // Update the message
    this->local_path_msg_.header.stamp = ros::Time::now();
    this->local_path_msg_.poses = std::vector<geometry_msgs::PoseStamped>(start, end);
    this->pub_local_path_.publish(this->local_path_msg_);
    this->pose_world_goal_ = this->local_path_msg_.poses[n_wp_prev].pose;
  }
}

int PathPublisherNode::closestWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start = 0)
{
  double min_dist = DBL_MAX;
  int id_closest = id_start;
  for (int i = id_start; i < path.poses.size(); i++)
  {
    const double dist = std::hypot(robot_pose.position.x - path.poses[i].pose.position.x, robot_pose.position.y - path.poses[i].pose.position.y);

    if (dist <= min_dist)
    {
      min_dist = dist;
      id_closest = i;
    }
    else
    {
      break;
    }
  }

  return id_closest;
}

int PathPublisherNode::nextWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const int id_start = 0)
{
  int id_closest = closestWaypoint(robot_pose, path, id_start);
  double yaw_T_robot_wp = atan2((path.poses[id_closest].pose.position.y - robot_pose.position.y),
                                (path.poses[id_closest].pose.position.x - robot_pose.position.x));

  const double yaw_robot = getYawFromOrientation(robot_pose.orientation); // 获得机器人朝向yaw
  const double angle = std::fabs(yaw_robot - yaw_T_robot_wp);
  const double angle_norm = std::min(2 * M_PI - angle, angle); // TODO: check if this is correct
  // 逻辑： 判断下和最近的点朝向相差大不大，如果大说明最近点不是下一个点，已经经过了，所以++
  if (angle_norm > M_PI / 2)
  {
    id_closest++;
  }
  // 否则下个点就是最近点
  return id_closest;
}

double PathPublisherNode::getYawFromOrientation(const geometry_msgs::Quaternion &orientation)
{
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  const tf2::Matrix3x3 m = tf2::Matrix3x3(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

// 将 geometry_msgs::Pose 类型的数据转换为 tf2::Transform 类型，
// 以便在ROS的TF框架中进行坐标变换等操作
tf2::Transform PathPublisherNode::convertPoseToTransform(const geometry_msgs::Pose &pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
}

std::pair<double, double> PathPublisherNode::calculatePoseError(const geometry_msgs::Pose &pose_robot, const geometry_msgs::Pose &pose_goal)
{
  // Positional Error 位置误差
  const double position_error = std::hypot(
    pose_robot.position.x - pose_goal.position.x,
    pose_robot.position.y - pose_goal.position.y
  );

  // Heading Error 朝向误差
  tf2::Quaternion q_robot, q_wp;
  // geometry_msgs::Pose pose_robot
  // 所以先通过tf2::fromMsg将朝向orientation转成tf2::Quaternion，方便下面旋转矩阵和欧拉角计算
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_wp);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_wp);

  double roll, pitch, yaw_robot, yaw_wp;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_wp);

  // 归一化角度到-180～180
  const double heading_error = unifyAngleRange(yaw_robot - yaw_wp) / M_PI * 180.0;

  return std::pair<double, double>(
    position_error,
    isLegal(heading_error)? heading_error : 0.0
  );
}

} // namespace me5413_world

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_publisher_node");
  me5413_world::PathPublisherNode path_publisher_node;
  ros::spin(); // spin the ros node.
  return 0;
}
