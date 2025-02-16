#include <cmath>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/logging.hpp>
#include "brains2/common/marker_color.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/msg/pose.hpp"
#include "brains2/msg/track_estimate.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std;
using namespace brains2::msg;
using namespace brains2::common;
using rclcpp::Publisher;
using rclcpp::Subscription;
using visualization_msgs::msg::MarkerArray;

class FakeTrackEstimationNode : public rclcpp::Node {
private:
    Subscription<Pose>::SharedPtr pose_sub;
    Publisher<TrackEstimate>::SharedPtr track_estimate_pub;
    Publisher<MarkerArray>::SharedPtr viz_pub;

    double last_s = 0.0;
    tl::optional<Track> track;
    TrackEstimate track_estimate_msg;
    MarkerArray viz_msg;

    void on_pose(const Pose::SharedPtr msg) {
        // Project pose onto track
        auto [s_proj, _] = track->project(Eigen::Vector2d(msg->x, msg->y), this->last_s, 5.0);
        this->last_s = std::fmod(s_proj, this->track->length() / 3);

        // Take 5m behind and 10 in front
        size_t start_id = track->find_interval(s_proj - 5), proj_id = track->find_interval(s_proj),
               end_id = track->find_interval(s_proj + 10) + 1;

        IC(s_proj, start_id, proj_id, end_id);

        // Update track estimate message
        track_estimate_msg.header.stamp = this->now();
        track_estimate_msg.header.frame_id = "world";
        track_estimate_msg.s_cen.resize(end_id - start_id);
        std::copy(track->get_vals_s().data() + start_id,
                  track->get_vals_s().data() + end_id,
                  track_estimate_msg.s_cen.begin());
        track_estimate_msg.x_cen.resize(end_id - start_id);
        std::copy(track->get_vals_X().data() + start_id,
                  track->get_vals_X().data() + end_id,
                  track_estimate_msg.x_cen.begin());
        track_estimate_msg.y_cen.resize(end_id - start_id);
        std::copy(track->get_vals_Y().data() + start_id,
                  track->get_vals_Y().data() + end_id,
                  track_estimate_msg.y_cen.begin());
        track_estimate_msg.phi_cen.resize(end_id - start_id);
        std::copy(track->get_vals_phi().data() + start_id,
                  track->get_vals_phi().data() + end_id,
                  track_estimate_msg.phi_cen.begin());
        track_estimate_msg.kappa_cen.resize(end_id - start_id);
        std::copy(track->get_vals_kappa().data() + start_id,
                  track->get_vals_kappa().data() + end_id,
                  track_estimate_msg.kappa_cen.begin());
        track_estimate_msg.w_cen.resize(end_id - start_id);
        std::copy(track->get_vals_width().data() + start_id,
                  track->get_vals_width().data() + end_id,
                  track_estimate_msg.w_cen.begin());
        // Publish track estimate
        track_estimate_pub->publish(track_estimate_msg);

        // Create center line message
        viz_msg.markers.resize(1);
        viz_msg.markers[0].header.frame_id = "world";
        viz_msg.markers[0].ns = "center_line";
        viz_msg.markers[0].action = visualization_msgs::msg::Marker::MODIFY;
        viz_msg.markers[0].type = visualization_msgs::msg::Marker::LINE_STRIP;
        viz_msg.markers[0].points.resize(track_estimate_msg.s_cen.size());
        for (size_t i = 0; i < track_estimate_msg.s_cen.size(); ++i) {
            viz_msg.markers[0].points[i].x = track_estimate_msg.x_cen[i];
            viz_msg.markers[0].points[i].y = track_estimate_msg.y_cen[i];
        }
        viz_msg.markers[0].scale.x = 0.05;
        viz_msg.markers[0].color = brains2::common::marker_colors("purple");

        // Publish it
        viz_pub->publish(viz_msg);
    }

public:
    FakeTrackEstimationNode() : Node("fake_track_estimation_node"), track_estimate_msg{} {
        auto track_name = this->declare_parameter<std::string>("track_name", "alpha");

        // Load track
#ifdef TRACK_DATABASE_PATH
        std::filesystem::path center_line_file(TRACK_DATABASE_PATH);
        center_line_file /= (track_name + "_center_line.csv");
        if (!std::filesystem::exists(center_line_file)) {
            throw std::runtime_error("Track " + track_name + " not found in TRACK_DATABASE_PATH");
        }
        this->track = Track::from_file(center_line_file.string());
        if (!this->track.has_value()) {
            throw std::runtime_error(
                "Track " + track_name +
                " could not be loaded because not all columns have the same length");
        }
#else
#error TRACK_DATABASE_PATH is not defined
#endif

        // create publisher and subscriber
        this->track_estimate_pub =
            this->create_publisher<TrackEstimate>("/brains2/track_estimate", 10);
        this->viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/brains2/viz/track_estimation",
            10);
        this->pose_sub = this->create_subscription<Pose>(
            "/brains2/pose",
            10,
            std::bind(&FakeTrackEstimationNode::on_pose, this, std::placeholders::_1));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeTrackEstimationNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
