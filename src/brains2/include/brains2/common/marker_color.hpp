// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef MARKER_COLOR_HPP
#define MARKER_COLOR_HPP

#include "std_msgs/msg/color_rgba.hpp"

namespace brains2 {
namespace common {

std_msgs::msg::ColorRGBA marker_colors(const std::string &color);

}  // namespace common
}  // namespace brains2

#endif  // MARKER_COLOR_HPP
