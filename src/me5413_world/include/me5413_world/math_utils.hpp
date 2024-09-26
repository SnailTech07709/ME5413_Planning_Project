/** math_utils.h
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Commonly used math functions
*/

#pragma once

#include <cmath>
#include <algorithm>
#include <math.h>

namespace me5413_world
{

// Return PI
constexpr double pi() { return M_PI; }

// Convert degrees to radians
inline double deg2rad(const double x) { return x * pi() / 180; }

// Convert radians to degrees
inline double rad2deg(const double x) { return x * 180 / pi(); }

// Convert metre per second to kilometers per hour
inline double mps2kph(const double x) { return x * 3.6; }

// Convert kilometers per hour to meter per second
inline double kph2mps(const double x) { return x / 3.6; }

// Convert angle into range [-pi, +pi]
inline double unifyAngleRange(const double angle)
{
  auto new_angle = angle;
  while (new_angle > pi())
  {
    new_angle -= 2 * pi();
  }
  while (new_angle < -pi())
  {
    new_angle += 2 * pi();
  }
  return new_angle;
}

// Limit the value within [lower_bound, upper_bound]
// inline：假设函数体很小且频繁调用，编译器可能会选择将其在每个调用点直接展开，而不实际生成函数调用，从而减少开销
// inline 关键字仅仅是对编译器的一种建议，用于尝试优化函数的性能
inline double limitWithinRange(const double value, const double lower_bound, const double upper_bound)
{
  auto new_value = std::max(value, lower_bound);
  new_value = std::min(new_value, upper_bound);
  return new_value;
}

// Check if a value is legal (not nan or inf)
inline bool isLegal(const double x)
{
  return (std::isnan(x) || std::isinf(x))? false : true;
}

} // end of namespace me5413_world
