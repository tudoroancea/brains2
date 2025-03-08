// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef BRAINS2__COMMON__MATH_HPP_
#define BRAINS2__COMMON__MATH_HPP_

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace brains2 {
namespace common {

double clip(double n, double lower, double upper);
Eigen::VectorXd atan2(const Eigen::VectorXd &y, const Eigen::VectorXd &x);
double teds_projection(double x, double a);
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

#endif  // BRAINS2__COMMON__MATH_HPP_
