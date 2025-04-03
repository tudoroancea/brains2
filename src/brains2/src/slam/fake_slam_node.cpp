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

#include "brains2/common/track_database.hpp"
#include "brains2/msg/map.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

using brains2::msg::Map;
using rclcpp::Publisher;

class FakeSLAMNode : public rclcpp::Node {
private:
    Publisher<Map>::SharedPtr map_pub;
    rclcpp::TimerBase::SharedPtr timer;

    Map map_msg;

public:
    FakeSLAMNode() : Node("fake_slam_node"), map_msg{} {
        auto track_name = this->declare_parameter<std::string>("track_name", "alpha");
        auto cones_map = brains2::common::load_cones_from_track_database(track_name);
        if (!cones_map) {
            throw std::runtime_error("Could not load cones from track database.");
        }
        // create map message
        map_msg.header.frame_id = "world";
        map_msg.header.stamp = this->now();
        for (const auto &[color, cones] : cones_map.value()) {
            for (int i = 0; i < cones.rows(); ++i) {
                map_msg.x.push_back(cones(i, 0));
                map_msg.y.push_back(cones(i, 1));
                map_msg.color.push_back(static_cast<uint8_t>(color));
            }
        }
        // create publisher
        this->map_pub = this->create_publisher<Map>("/brains2/map", 10);
        const auto timer_period = 1 / this->declare_parameter<double>("freq", 10.0);
        this->timer = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                              [this]() { this->map_pub->publish(this->map_msg); });
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeSLAMNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
