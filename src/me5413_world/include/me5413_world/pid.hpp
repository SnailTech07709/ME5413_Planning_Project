/** pid.hpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Implementation of PID controller
 */

#pragma once

#include <iostream>
#include <cmath>
#include <queue>  // 用于队列


namespace control
{
class PID
{
 public:
  PID() {}; //构造函数
  PID(double dt, double max, double min, double Kp, double Kd, double Ki, int max_queue_size);  //构造函数
  ~PID() {};  //析构函数

  void updateSettings(const double Kp, const double Kd, const double Ki);  // 函数接口
  // Returns the manipulated variable given a setpoint and current process value
  double calculate(const double setpoint, const double pv);  //函数接口

 private:
  double dt_;
  double max_;
  double min_;
  double Kp_;
  double Kd_;
  double Ki_;
  double pre_error_;
  double integral_;
  // 改进部分
  // 使用队列存储最近的误差
  std::queue<double> error_queue_;
  int max_queue_size;  // 仅保存最近20个误差
};

// 构造函数实现
PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki,int max_queue_size) :
  dt_(dt),
  max_(max),
  min_(min),
  Kp_(Kp),
  Kd_(Kd),
  Ki_(Ki),
  pre_error_(0),
  integral_(0),
  max_queue_size(20)
{};

// 函数实现（kp, ki, kd赋值）
void PID::updateSettings(const double Kp, const double Kd, const double Ki)
{
  this->Kp_ = Kp;
  this->Kd_ = Kd;
  this->Ki_ = Ki;
};

// PID函数实现
double PID::calculate(const double setpoint, const double pv)
{
  // Calculate error
  double error = setpoint - pv;  // 计算误差

  // Proportional term
  const double P_term = Kp_ * error;

  // Integral term
  // 积分项：使用队列计算最近的误差和
  error_queue_.push(error * dt_);
  integral_ += error * dt_;  // 累积误差

  // 一直累积不太好, 所以就计算20个误差的累积
  if (error_queue_.size() > max_queue_size)
  {
    integral_ -= error_queue_.front();
    error_queue_.pop();
  }
  
  const double I_term = Ki_ * integral_;

  // Derivative term
  const double derivative = (error - pre_error_) / dt_;
  const double D_term = Kd_ * derivative;

  // Calculate total output
  double output = P_term + I_term + D_term;

  // Restrict to max/min
  output = std::min(output, max_);
  output = std::max(output, min_);

  // Save error to previous error
  pre_error_ = error;

  return output;
};

} // namespace control
