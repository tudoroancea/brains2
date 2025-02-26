// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include <cstdint>
#include <memory>
#include <tuple>
#include <type_traits>
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
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rmw/qos_profiles.h"
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

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        const auto dt = 1 / this->declare_parameter("freq", 20.0);
        const auto Nf = this->declare_parameter("Nf", 10);

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
            car_constants["actuators"]["steering_time_constant"].as<double>(),
        };

        // Limits
        const brains2::control::Controller::ConstraintsParams constraints_params{
            this->declare_parameter("v_x_max", 10.0),
            car_constants["actuators"]["steering_max"].as<double>(),
            car_constants["actuators"]["steering_rate_max"].as<double>(),
            this->declare_parameter("tau_max", 100.0),
            car_constants["geometry"]["car_width"].as<double>() +
                this->declare_parameter("car_width_inflation", 0.0)};
#else
#error CAR_CONSTANTS_PATH is not defined
#endif

        // cost params
        const brains2::control::Controller::CostParams cost_params{
            this->declare_parameter("v_ref", 5.0),
            this->declare_parameter("q_s", 1.0),
            this->declare_parameter("q_n", 1.0),
            this->declare_parameter("q_psi", 1.0),
            this->declare_parameter("q_v", 1.0),
            this->declare_parameter("r_delta", 1.0),
            this->declare_parameter("r_delta_dot", 1.0),
            this->declare_parameter("r_tau", 1.0),
            this->declare_parameter("q_s_f", 1.0),
            this->declare_parameter("q_n_f", 1.0),
            this->declare_parameter("q_psi_f", 1.0),
            this->declare_parameter("q_v_f", 1.0),
        };

        const brains2::control::Controller::SolverParams solver_params{
            this->declare_parameter("jit", false),
            this->declare_parameter("solver", "fatrop"),
        };

        // Create controller object
        this->controller = std::make_unique<brains2::control::Controller>(static_cast<size_t>(Nf),
                                                                          model_params,
                                                                          constraints_params,
                                                                          cost_params,
                                                                          solver_params);

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
        this->pose_sub = this->create_subscription<Pose>(
            "/brains2/pose",
            10,
            std::bind(&ControlNode::pose_cb, this, std::placeholders::_1));
        this->vel_sub = this->create_subscription<Velocity>(
            "/brains2/velocity",
            10,
            std::bind(&ControlNode::vel_cb, this, std::placeholders::_1));
        this->current_controls_sub = this->create_subscription<Controls>(
            "/brains2/current_controls",
            10,
            std::bind(&ControlNode::current_controls_cb, this, std::placeholders::_1));

        // Diagnostics
        this->diag_msg.status.resize(1);
        this->diag_msg.status[0].name = "control_node";
        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        this->diag_msg.status[0].message = "OK";
        this->diag_msg.status[0].values.resize(1);
        this->diag_msg.status[0].values[0].key = "runtime (ms)";

        // Markers
        this->viz_msg.markers.resize(2);

        this->viz_msg.markers[0].header.frame_id = "world";
        this->viz_msg.markers[0].ns = "traj_pred";
        this->viz_msg.markers[0].action = visualization_msgs::msg::Marker::MODIFY;
        this->viz_msg.markers[0].type = visualization_msgs::msg::Marker::LINE_STRIP;
        this->viz_msg.markers[0].points.resize(Nf + 1);
        this->viz_msg.markers[0].scale.x = 0.05;
        this->viz_msg.markers[0].color = marker_colors("green");

        this->viz_msg.markers[1].header.frame_id = "world";
        this->viz_msg.markers[1].ns = "traj_ref";
        this->viz_msg.markers[1].action = visualization_msgs::msg::Marker::MODIFY;
        this->viz_msg.markers[1].type = visualization_msgs::msg::Marker::LINE_STRIP;
        this->viz_msg.markers[1].points.resize(Nf + 1);
        this->viz_msg.markers[1].scale.x = 0.05;
        this->viz_msg.markers[1].color = marker_colors("orange");
    }

private:
    // publishers
    Publisher<Controls>::SharedPtr target_controls_pub;
    Publisher<MarkerArray>::SharedPtr viz_pub;
    Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;

    // subscribers
    Subscription<TrackEstimate>::SharedPtr track_estimate_sub;
    Subscription<Pose>::SharedPtr pose_sub;
    Subscription<Velocity>::SharedPtr vel_sub;
    Subscription<Controls>::SharedPtr current_controls_sub;

    // messages
    brains2::msg::Controls controls_msg;
    visualization_msgs::msg::MarkerArray viz_msg;
    diagnostic_msgs::msg::DiagnosticArray diag_msg;

    // controller
    unique_ptr<Track> track;
    Pose::ConstSharedPtr pose_msg;
    Velocity::ConstSharedPtr vel_msg;
    Controls::ConstSharedPtr current_controls_msg;
    rclcpp::TimerBase::SharedPtr control_timer;
    unique_ptr<brains2::control::Controller> controller;

    // Counter to throw an error after 5 consecutive errors
    uint8_t error_counter = 0;
    static constexpr uint8_t MAX_ERROR_COUNT = 5;

    void track_estimate_cb(const TrackEstimate::ConstSharedPtr msg) {
        const auto track_expected = Track::from_values(msg->s_cen,
                                                       msg->x_cen,
                                                       msg->y_cen,
                                                       msg->phi_cen,
                                                       msg->kappa_cen,
                                                       msg->w_cen);
        if (!track_expected) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Could not construct Track object from received message because of error: %s",
                to_string(track_expected.error()).c_str());
        } else {
            this->track = std::make_unique<Track>(track_expected.value());
        }
    }

    void pose_cb(const Pose::ConstSharedPtr pose_msg) {
        this->pose_msg = pose_msg;
        // We launch the control timer upon receiving the first pose message and not velocity
        // message because eventually (when we will have SLAM), the pose messages will likely be the
        // ones that will be "late".
        if (!this->control_timer) {
            this->control_timer = this->create_wall_timer(
                std::chrono::duration<double>(1 / this->get_parameter("freq").as_double()),
                std::bind(&ControlNode::control_cb, this));
        }
    }

    void vel_cb(const Velocity::ConstSharedPtr vel_msg) {
        this->vel_msg = vel_msg;
    }

    void current_controls_cb(const Controls::ConstSharedPtr controls_msg) {
        this->current_controls_msg = controls_msg;
    }

    void control_cb() {
        if (!this->pose_msg || !this->vel_msg || !this->current_controls_msg) {
            RCLCPP_INFO(this->get_logger(),
                        "Pose, velocity, or current controls message not received; skipping "
                        "control computation");
            return;
        }
        if (!this->track) {
            RCLCPP_INFO(this->get_logger(), "Track not initialized; skipping control computation");
            return;
        }
        // Convert CartesianPose to FrenetPose
        const auto frenet_pose =
            track->cartesian_to_frenet(CartesianPose{pose_msg->x, pose_msg->y, pose_msg->phi})
                .first;

        // Construct current state
        const Controller::State state{frenet_pose.s,
                                      frenet_pose.n,
                                      frenet_pose.psi,
                                      std::hypot(vel_msg->v_x, vel_msg->v_y),
                                      current_controls_msg->delta};

        // Compute control
        const auto start = this->now();

        const auto control = this->controller->compute_control(state, *(this->track));
        if (control) {
            error_counter = 0;
            const auto sim_control = to_sim_control(*control);
            controls_msg.tau_fl = sim_control.u_tau_FL;
            controls_msg.tau_fr = sim_control.u_tau_FR;
            controls_msg.tau_rl = sim_control.u_tau_RL;
            controls_msg.tau_rr = sim_control.u_tau_RR;
            controls_msg.delta = sim_control.u_delta;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Error in MPC solver: %s; sending the old control",
                         to_string(control.error()).c_str());
            ++error_counter;
            if (error_counter >= MAX_ERROR_COUNT) {
                controls_msg.tau_fl = 0.0;
                controls_msg.tau_fr = 0.0;
                controls_msg.tau_rl = 0.0;
                controls_msg.tau_rr = 0.0;
                controls_msg.delta = 0.0;
                this->target_controls_pub->publish(controls_msg);
                throw std::runtime_error("Failed to compute control " + to_string(MAX_ERROR_COUNT) +
                                         " times in a row. Aborting.");
            }
        }
        const auto end = this->now();

        // Publish controls
        controls_msg.header.stamp = this->now();
        this->target_controls_pub->publish(controls_msg);

        // Publish diagnostics
        diag_msg.header.stamp = this->now();
        diag_msg.status[0].values[0].value = to_string(1000 * (end - start).seconds());
        diagnostics_pub->publish(diag_msg);

        // Publish visualization
        const auto x_opt = this->controller->get_x_opt();
        const auto x_ref = this->controller->get_x_ref();
        for (long i = 0; i < x_opt.cols(); ++i) {
            // predicted trajectory
            const auto [pose, _] = track->frenet_to_cartesian(
                FrenetPose{frenet_pose.s + x_opt(0, i), x_opt(1, i), x_opt(2, i)});
            viz_msg.markers[0].points[i].x = pose.X;
            viz_msg.markers[0].points[i].y = pose.Y;
            viz_msg.markers[0].points[i].z = 0.1;

            // reference trajectory
            const auto s_ref = frenet_pose.s + x_ref(0, i), X_ref = track->eval_X(s_ref),
                       Y_ref = track->eval_Y(s_ref);
            viz_msg.markers[1].points[i].x = X_ref;
            viz_msg.markers[1].points[i].y = Y_ref;
            viz_msg.markers[1].points[i].z = 0.05;
        }
        viz_pub->publish(viz_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
