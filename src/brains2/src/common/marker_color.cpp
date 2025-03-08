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

#include "brains2/common/marker_color.hpp"

std_msgs::msg::ColorRGBA brains2::common::marker_colors(const std::string& color) {
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.a = 1.0;
    if (color == "red") {
        color_msg.r = 1.0;
    } else if (color == "green") {
        color_msg.g = 1.0;
    } else if (color == "blue") {
        color_msg.b = 1.0;
    } else if (color == "yellow") {
        color_msg.r = 1.0;
        color_msg.g = 1.0;
    } else if (color == "orange") {
        color_msg.r = 1.0;
        color_msg.g = 0.5;
    } else if (color == "purple") {
        color_msg.r = 0.5;
        color_msg.b = 0.5;
    } else if (color == "magenta") {
        color_msg.r = 1.0;
        color_msg.b = 1.0;
    } else if (color == "cyan") {
        color_msg.g = 1.0;
        color_msg.b = 1.0;
    } else if (color == "light_blue") {
        color_msg.g = 0.5;
        color_msg.b = 1.0;
    } else if (color == "dark_blue") {
        color_msg.b = 0.5;
    } else if (color == "brown") {
        color_msg.r = 0.5;
        color_msg.g = 0.25;
    } else if (color == "white") {
        color_msg.r = 1.0;
        color_msg.g = 1.0;
        color_msg.b = 1.0;
    } else if (color == "gray") {
        color_msg.r = 0.5;
        color_msg.g = 0.5;
        color_msg.b = 0.5;
    } else if (color == "light_gray") {
        color_msg.r = 0.75;
        color_msg.g = 0.75;
        color_msg.b = 0.75;
    } else if (color == "dark_gray") {
        color_msg.r = 0.25;
        color_msg.g = 0.25;
        color_msg.b = 0.25;
    } else if (color == "black") {
        color_msg.r = 0.0;
        color_msg.g = 0.0;
        color_msg.b = 0.0;
    } else {
        color_msg.r = 1.0;
        color_msg.g = 1.0;
        color_msg.b = 1.0;
    }
    return color_msg;
}
