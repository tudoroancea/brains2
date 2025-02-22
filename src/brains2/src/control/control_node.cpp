// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include <rmw/qos_profiles.h>
#include <memory>
#include <tuple>
#include "brains2/common/cone_color.hpp"
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/control/controller.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/msg/controls.hpp"
#include "brains2/msg/detail/velocity__struct.hpp"
#include "brains2/msg/pose.hpp"
#include "brains2/msg/track_estimate.hpp"
#include "brains2/msg/velocity.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace brains2::msg;
using namespace brains2::common;
using namespace brains2::control;
using rclcpp::Publisher;
using rclcpp::Subscription;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

typedef message_filters::sync_policies::ApproximateTime<Pose, Velocity> StateMsgSyncPolicy;
typedef message_filters::Synchronizer<StateMsgSyncPolicy> StateMsgSync;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        const auto dt = 1 / this->declare_parameter("freq", 20.0);
        const auto Nf = this->declare_parameter("Nf", 10);

        // Limits
        const brains2::control::Controller::Limits limits{
            this->declare_parameter("v_x_max", 10.0),
            this->declare_parameter("delta_max", 0.5),
            this->declare_parameter("tau_max", 100.0)};

        // load yaml file with car constants
#ifdef CAR_CONSTANTS_PATH
        YAML::Node car_constants = YAML::LoadFile(CAR_CONSTANTS_PATH);
        const brains2::control::Controller::ModelParams model_params{
            dt,
            car_constants["inertia"]["mass"].as<double>(),
            car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
            car_constants["geometry"]["wheelbase"].as<double>() -
                car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
            car_constants["drivetrain"]["C_m0"].as<double>(),
            car_constants["drivetrain"]["C_r0"].as<double>(),
            car_constants["drivetrain"]["C_r1"].as<double>(),
            car_constants["drivetrain"]["C_r2"].as<double>(),
        };

#else
#error CAR_CONSTANTS_PATH is not defined
#endif

        // cost params
        const brains2::control::Controller::CostParams cost_params{
            .v_ref = this->declare_parameter("v_ref", 5.0),
            .delta_s_ref = this->declare_parameter("delta_s_ref", 5.0),
            .q_s = this->declare_parameter("q_s", 1.0),
            .q_n = this->declare_parameter("q_n", 1.0),
            .q_psi = this->declare_parameter("q_psi", 1.0),
            .q_v = this->declare_parameter("q_v", 1.0),
            .r_delta = this->declare_parameter("r_delta", 1.0),
            .r_tau = this->declare_parameter("r_tau", 1.0),
            .q_s_f = this->declare_parameter("q_s_f", 1.0),
            .q_n_f = this->declare_parameter("q_n_f", 1.0),
            .q_psi_f = this->declare_parameter("q_psi_f", 1.0),
            .q_v_f = this->declare_parameter("q_v_f", 1.0),
        };

        // Create controller object
        this->controller =
            std::make_unique<brains2::control::Controller>(static_cast<size_t>(Nf),
                                                           model_params,
                                                           limits,
                                                           cost_params,
                                                           1,
                                                           this->declare_parameter("jit", false));

        // Publishers and subscribers
        this->target_controls_pub =
            this->create_publisher<Controls>("/brains2/target_controls", 10);
        this->viz_pub =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/brains2/viz/control",
                                                                         10);
        this->diagnostics_pub =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/brains2/diagnostics",
                                                                          10);
        this->track_estimate_sub = this->create_subscription<TrackEstimate>(
            "/brains2/track_estimate",
            10,
            std::bind(&ControlNode::track_estimate_cb, this, std::placeholders::_1));
        this->pose_sub =
            std::make_shared<message_filters::Subscriber<Pose>>(this,
                                                                "/brains2/pose",
                                                                rmw_qos_profile_default);
        this->vel_sub =
            std::make_shared<message_filters::Subscriber<Velocity>>(this,
                                                                    "/brains2/velocity",
                                                                    rmw_qos_profile_default);
        this->sync =
            std::make_shared<StateMsgSync>(StateMsgSyncPolicy(10), *this->pose_sub, *this->vel_sub);
        this->sync->registerCallback(&ControlNode::state_cb, this);

        // Diagnostics
        this->diag_msg.status.resize(1);
        this->diag_msg.status[0].name = "control_node";
        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        this->diag_msg.status[0].message = "OK";
        this->diag_msg.status[0].values.resize(1);
        this->diag_msg.status[0].values[0].key = "runtime (ms)";
    }

private:
    // publishers
    Publisher<Controls>::SharedPtr target_controls_pub;
    Publisher<MarkerArray>::SharedPtr viz_pub;
    Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;

    // subscribers
    Subscription<TrackEstimate>::SharedPtr track_estimate_sub;
    shared_ptr<message_filters::Subscriber<Pose>> pose_sub;
    shared_ptr<message_filters::Subscriber<Velocity>> vel_sub;
    shared_ptr<StateMsgSync> sync;

    // messages
    brains2::msg::Controls controls_msg;
    visualization_msgs::msg::MarkerArray viz_msg;
    diagnostic_msgs::msg::DiagnosticArray diag_msg;

    // controller
    std::unique_ptr<Track> track;
    std::unique_ptr<brains2::control::Controller> controller;

    // only treat one in 5 messages
    uint8_t counter = 0;

    void track_estimate_cb(const TrackEstimate::SharedPtr msg) {
        const auto track_expected = Track::from_values(msg->s_cen,
                                                       msg->x_cen,
                                                       msg->y_cen,
                                                       msg->phi_cen,
                                                       msg->kappa_cen,
                                                       msg->w_cen);
        if (!track_expected) {
            RCLCPP_ERROR(this->get_logger(),
                         "Could not construct Track object from received message (arrays with "
                         "different lengths ?)");
        }
        this->track = std::make_unique<Track>(track_expected.value());
    }

    void state_cb(const Pose::SharedPtr pose_msg, const Velocity::SharedPtr vel_msg) {
        if (!this->track) {
            return;
        }
        if (this->counter == 0) {
            // Construct current state
            const Controller::State state{.X = pose_msg->x,
                                          .Y = pose_msg->y,
                                          .phi = pose_msg->phi,
                                          .v = std::hypot(vel_msg->v_x, vel_msg->v_y)};

            auto start = this->now();
            const auto controls = this->controller->compute_control(state, *(this->track));
            auto end = this->now();
            if (!controls.has_value()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Error in MPC solver: %s",
                             brains2::control::Controller::to_string(controls.error()).c_str());
            }

            // Publish controls
            controls_msg.header.stamp = this->now();
            controls_msg.tau_fl = controls->tau / 4;
            controls_msg.tau_fr = controls->tau / 4;
            controls_msg.tau_rl = controls->tau / 4;
            controls_msg.tau_rr = controls->tau / 4;
            controls_msg.delta = controls->delta;
            this->target_controls_pub->publish(controls_msg);

            // Publish diagnostics
            diag_msg.status[0].values[0].value = std::to_string(1000 * (end - start).seconds());
            diagnostics_pub->publish(diag_msg);

            // TODO: add viz
            viz_msg.markers.resize(1);
            viz_msg.markers[0].header.frame_id = "world";
            viz_msg.markers[0].ns = "traj_pred";
            viz_msg.markers[0].action = visualization_msgs::msg::Marker::MODIFY;
            viz_msg.markers[0].type = visualization_msgs::msg::Marker::LINE_STRIP;
            const auto x_opt = this->controller->get_x_opt();
            const auto npoints = x_opt.cols();
            viz_msg.markers[0].points.resize(npoints);
            for (long i = 0; i < npoints; ++i) {
                auto [X, Y, phi] =
                    track->frenet_to_cartesian(x_opt(0, i), x_opt(1, i), x_opt(2, i));
                viz_msg.markers[0].points[i].x = X;
                viz_msg.markers[0].points[i].y = Y;
            }
            viz_msg.markers[0].scale.x = 0.05;
            viz_msg.markers[0].color = marker_colors("green");
            this->viz_pub->publish(viz_msg);
        }
        this->counter = (this->counter + 1) % 5;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
