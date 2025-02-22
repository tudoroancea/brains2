#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/logging.hpp>
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/external/expected.hpp"
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
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class FakeTrackEstimationNode : public rclcpp::Node {
private:
    Subscription<Pose>::SharedPtr pose_sub;
    Publisher<TrackEstimate>::SharedPtr track_estimate_pub;
    Publisher<MarkerArray>::SharedPtr viz_pub;
    rclcpp::TimerBase::SharedPtr timer;

    double last_s = 0.0;
    unique_ptr<Track> track;
    TrackEstimate track_estimate_msg;
    MarkerArray viz_msg;
    shared_ptr<Pose> last_pose;

    void on_pose(const Pose::SharedPtr msg) {
        last_pose = msg;
    }
    void timer_cb() {
        if (!last_pose) {
            return;
        }
        // Project pose onto track
        auto [s_proj, _] = track->project(last_pose->x, last_pose->y, this->last_s, 30.0);

        // Update last s value (don't forget the Track here represents 3 laps)
        this->last_s = std::fmod(s_proj, track->length() / 3);

        // Take 5m behind and 10 in front
        size_t start_id =
                   track->find_interval(s_proj - this->get_parameter("dist_back").as_double()),
               end_id =
                   track->find_interval(s_proj + this->get_parameter("dist_front").as_double()) + 1;
        const auto npoints = end_id - start_id;

        // Update track estimate message
        track_estimate_msg.header.stamp = this->now();
        track_estimate_msg.header.frame_id = "world";
        track_estimate_msg.s_cen.resize(npoints);
        std::copy(track->get_vals_s().data() + start_id,
                  track->get_vals_s().data() + end_id,
                  track_estimate_msg.s_cen.begin());
        track_estimate_msg.x_cen.resize(npoints);
        std::copy(track->get_vals_X().data() + start_id,
                  track->get_vals_X().data() + end_id,
                  track_estimate_msg.x_cen.begin());
        track_estimate_msg.y_cen.resize(npoints);
        std::copy(track->get_vals_Y().data() + start_id,
                  track->get_vals_Y().data() + end_id,
                  track_estimate_msg.y_cen.begin());
        track_estimate_msg.phi_cen.resize(npoints);
        std::copy(track->get_vals_phi().data() + start_id,
                  track->get_vals_phi().data() + end_id,
                  track_estimate_msg.phi_cen.begin());
        track_estimate_msg.kappa_cen.resize(npoints);
        std::copy(track->get_vals_kappa().data() + start_id,
                  track->get_vals_kappa().data() + end_id,
                  track_estimate_msg.kappa_cen.begin());
        track_estimate_msg.w_cen.resize(npoints);
        std::copy(track->get_vals_width().data() + start_id,
                  track->get_vals_width().data() + end_id,
                  track_estimate_msg.w_cen.begin());

        // post-process the heading values phi such that they all lie "around" current phi
        // for this, wrap all in interval [phi-pi, phi+pi) and then remove discontinuities
        const auto tpr = last_pose->phi - M_PI;
        for (auto &phi_cen : track_estimate_msg.phi_cen) {
            phi_cen = teds_projection(phi_cen, tpr);
        }
        std::vector<double> diffs(npoints);
        std::adjacent_difference(track_estimate_msg.phi_cen.begin(),
                                 track_estimate_msg.phi_cen.end(),
                                 diffs.begin());
        if (std::any_of(diffs.begin(), diffs.end(), [](auto x) { return x > M_PI; })) {
            throw std::runtime_error("Heading discontinuity detected in local track estimation");
        }

        // Publish track estimate
        track_estimate_pub->publish(track_estimate_msg);

        // Create center line message
        viz_msg.markers.resize(1);
        viz_msg.markers[0].header.frame_id = "world";
        viz_msg.markers[0].ns = "center_line";
        viz_msg.markers[0].action = visualization_msgs::msg::Marker::MODIFY;
        viz_msg.markers[0].type = visualization_msgs::msg::Marker::LINE_STRIP;
        // TODO: maybe only send a subset of the points to have a rougher
        // but faster visualization?
        viz_msg.markers[0].points.resize(npoints);
        for (size_t i = 0; i < npoints; ++i) {
            viz_msg.markers[0].points[i].x = track_estimate_msg.x_cen[i];
            viz_msg.markers[0].points[i].y = track_estimate_msg.y_cen[i];
        }
        viz_msg.markers[0].scale.x = 0.05;
        viz_msg.markers[0].color = brains2::common::marker_colors("purple");

        // Publish it
        viz_pub->publish(viz_msg);
    }

public:
    FakeTrackEstimationNode()
        : Node("fake_track_estimation_node"), track_estimate_msg{}, viz_msg{}, last_pose(nullptr) {
        this->declare_parameter("dist_front", 10.0);
        this->declare_parameter("dist_back", 5.0);

        // Load track
#ifdef TRACK_DATABASE_PATH
        std::filesystem::path center_line_file(TRACK_DATABASE_PATH);
        const auto track_name = this->declare_parameter("track_name", "alpha");
        center_line_file /= (track_name + "_center_line.csv");
        if (!std::filesystem::exists(center_line_file)) {
            throw std::runtime_error("Track " + track_name + " not found in TRACK_DATABASE_PATH");
        }
        const auto track_expected = Track::from_file(center_line_file.string());
        if (!track_expected.has_value()) {
            throw std::runtime_error(
                "Track " + track_name +
                " could not be loaded because not all columns have the same length");
        }
        // NOTE: this will effectively copy the instance of Track. This is necessary because the
        // previous object was allocated on the stack and will be destroyed at the end of the
        // constructor.
        this->track = std::make_unique<Track>(track_expected.value());
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

        // Create timer
        const auto timer_period = 1 / this->declare_parameter<double>("freq", 10.0);
        this->timer = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                              bind(&FakeTrackEstimationNode::timer_cb, this));
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
