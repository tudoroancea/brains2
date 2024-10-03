// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef MATH_HPP
#define MATH_HPP

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace brains2 {
namespace common {

double clip(double n, double lower, double upper);
Eigen::VectorXd atan2(const Eigen::VectorXd &y, const Eigen::VectorXd &x);
double wrap_to_pi(double x);
Eigen::VectorXd wrap_to_pi(const Eigen::VectorXd &x);
float deg2rad(float deg);
double deg2rad(double deg);
float rad2deg(float rad);
double rad2deg(double rad);
tf2::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
geometry_msgs::msg::Quaternion rpy_to_quaternion_msg(double roll, double pitch, double yaw);

}  // namespace common
}  // namespace brains2

#endif  // MATH_HPP
