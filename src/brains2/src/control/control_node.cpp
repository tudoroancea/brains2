// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>
#include <tuple>
#include "brains2/common/cone_color.hpp"
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/control/controller.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/msg/controller_debug_info.hpp"
#include "brains2/msg/controls.hpp"
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
using diagnostic_msgs::msg::DiagnosticArray;
using rclcpp::Publisher;
using rclcpp::Subscription;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        const auto dt = 1 / this->declare_parameter("freq", 20.0);
        const auto Nf = this->declare_parameter("Nf", 20);

        // load yaml file with car constants
#ifdef CAR_CONSTANTS_PATH
        YAML::Node car_constants = YAML::LoadFile(CAR_CONSTANTS_PATH);
        const Controller::ModelParams model_params{
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

        // Limits
        const Controller::ConstraintsParams constraints_params{
            this->declare_parameter("v_x_max", 10.0),
            this->declare_parameter("delta_max", 0.5),
            this->declare_parameter("tau_max", 100.0),
            car_constants["geometry"]["car_width"].as<double>() +
                this->declare_parameter("car_width_inflation", 0.0)};
        this->car_width = car_constants["geometry"]["car_width"].as<double>();
#else
#error CAR_CONSTANTS_PATH is not defined
#endif

        // cost params
        const Controller::CostParams cost_params{
            this->declare_parameter("v_ref", 5.0),
            this->declare_parameter("q_s", 1.0),
            this->declare_parameter("q_n", 1.0),
            this->declare_parameter("q_psi", 1.0),
            this->declare_parameter("q_v", 1.0),
            this->declare_parameter("r_delta", 1.0),
            this->declare_parameter("r_tau", 1.0),
            this->declare_parameter("q_s_f", 1.0),
            this->declare_parameter("q_n_f", 1.0),
            this->declare_parameter("q_psi_f", 1.0),
            this->declare_parameter("q_v_f", 1.0),
        };

        const Controller::SolverParams solver_params{
            this->declare_parameter("jit", false),
            this->declare_parameter("solver", "fatrop"),
        };

        // Create controller object
        this->controller = std::make_unique<Controller>(static_cast<size_t>(Nf),
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
        this->debug_info_pub =
            this->create_publisher<ControllerDebugInfo>("/brains2/control/debug_info", 10);
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

        // Initialize messages
        this->create_viz_msg();
        this->create_diag_msg();
        this->create_debug_info_msg();
    }

private:
    /******************************************************************************
     * Input data
     ******************************************************************************/

    // Pose
    Subscription<Pose>::SharedPtr pose_sub;
    Pose::ConstSharedPtr pose_msg;

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

    // Velocity
    Subscription<Velocity>::SharedPtr vel_sub;
    Velocity::ConstSharedPtr vel_msg;

    void vel_cb(const Velocity::ConstSharedPtr vel_msg) {
        this->vel_msg = vel_msg;
    }

    // Track Estimate
    Subscription<TrackEstimate>::SharedPtr track_estimate_sub;
    unique_ptr<Track> track;

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
            this->track = std::make_unique<Track>(*track_expected);
        }
    }

    /******************************************************************************
     * Output data
     ******************************************************************************/
    Publisher<Controls>::SharedPtr target_controls_pub;
    Controls controls_msg;
    rclcpp::TimerBase::SharedPtr control_timer;
    unique_ptr<Controller> controller;
    double car_width;
    // Counter to throw an error after 5 consecutive errors
    uint8_t error_counter = 0;
    static constexpr uint8_t MAX_ERROR_COUNT = 5;

    void control_cb() {
        if (!this->pose_msg || !this->vel_msg) {
            RCLCPP_INFO(this->get_logger(),
                        "Pose or velocity message not received; skipping control computation");
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
                                      std::hypot(vel_msg->v_x, vel_msg->v_y)};

        // Compute control
        const auto start = this->now();
        const auto control = this->controller->compute_control(state, *(this->track))
                                 .transform([this](const auto &control) {
                                     error_counter = 0;
                                     return to_sim_control(control);
                                 })
                                 .transform_error([this](const auto &error) {
                                     RCLCPP_ERROR(this->get_logger(),
                                                  "Error in MPC solver: %s",
                                                  to_string(error).c_str());
                                     ++error_counter;
                                     if (error_counter >= MAX_ERROR_COUNT) {
                                         throw std::runtime_error("Failed to compute control " +
                                                                  to_string(MAX_ERROR_COUNT) +
                                                                  " times in a row. Aborting.");
                                     }
                                     return error;
                                 })
                                 .value_or(brains2::sim::Sim::Control{0, 0, 0, 0, 0});
        const auto end = this->now();

        // Publish controls
        this->controls_msg.header.stamp = this->now();
        this->controls_msg.tau_fl = control.u_tau_FL;
        this->controls_msg.tau_fr = control.u_tau_FR;
        this->controls_msg.tau_rl = control.u_tau_RL;
        this->controls_msg.tau_rr = control.u_tau_RR;
        this->controls_msg.delta = control.u_delta;
        this->target_controls_pub->publish(this->controls_msg);

        // Publish diagnostics
        this->update_diag_msg(1000 * (end - start).seconds());
        this->diagnostics_pub->publish(this->diag_msg);

        // Publish visualization
        this->update_viz_msg(frenet_pose.s,
                             this->controller->get_x_ref(),
                             this->controller->get_x_opt());
        this->viz_pub->publish(this->viz_msg);

        // Publish other debug information
        this->update_debug_info_msg(this->controller->get_x_ref(),
                                    this->controller->get_u_ref(),
                                    this->controller->get_x_opt(),
                                    this->controller->get_u_opt());
        this->debug_info_pub->publish(this->debug_info_msg);
    }

    Publisher<MarkerArray>::SharedPtr viz_pub;
    MarkerArray viz_msg;

    void create_viz_msg() {
        this->viz_msg.markers.resize(4);

        // Predicted trajectory
        this->viz_msg.markers[0].header.frame_id = "world";
        this->viz_msg.markers[0].ns = "traj_pred";
        this->viz_msg.markers[0].action = visualization_msgs::msg::Marker::MODIFY;
        this->viz_msg.markers[0].type = visualization_msgs::msg::Marker::LINE_STRIP;
        this->viz_msg.markers[0].points.resize(this->controller->horizon_size() + 1);
        this->viz_msg.markers[0].scale.x = 0.05;
        this->viz_msg.markers[0].color = marker_colors("green");

        // Reference trajectory
        this->viz_msg.markers[1].header.frame_id = "world";
        this->viz_msg.markers[1].ns = "traj_ref";
        this->viz_msg.markers[1].action = visualization_msgs::msg::Marker::MODIFY;
        this->viz_msg.markers[1].type = visualization_msgs::msg::Marker::LINE_STRIP;
        this->viz_msg.markers[1].points.resize(this->controller->horizon_size() + 1);
        this->viz_msg.markers[1].scale.x = 0.05;
        this->viz_msg.markers[1].color = marker_colors("orange");

        // Adjusted left boundary (blue)
        this->viz_msg.markers[2].header.frame_id = "world";
        this->viz_msg.markers[2].ns = "boundary_left";
        this->viz_msg.markers[2].action = visualization_msgs::msg::Marker::MODIFY;
        this->viz_msg.markers[2].type = visualization_msgs::msg::Marker::LINE_STRIP;
        this->viz_msg.markers[2].points.resize(this->controller->horizon_size() + 1);
        this->viz_msg.markers[2].scale.x = 0.05;
        this->viz_msg.markers[2].color = marker_colors("blue");

        // Adjusted right boundary (yellow)
        this->viz_msg.markers[3].header.frame_id = "world";
        this->viz_msg.markers[3].ns = "boundary_right";
        this->viz_msg.markers[3].action = visualization_msgs::msg::Marker::MODIFY;
        this->viz_msg.markers[3].type = visualization_msgs::msg::Marker::LINE_STRIP;
        this->viz_msg.markers[3].points.resize(this->controller->horizon_size() + 1);
        this->viz_msg.markers[3].scale.x = 0.05;
        this->viz_msg.markers[3].color = marker_colors("yellow");
    }

    void update_viz_msg(const double s,
                        const Controller::StateHorizonMatrix &x_ref,
                        const Controller::StateHorizonMatrix &x_opt) {
        if (!track) {
            return;
        }
        for (long i = 0; i < this->controller->horizon_size() + 1; ++i) {
            // reference trajectory
            const auto s_ref = s + x_ref(0, i), X_ref = track->eval_X(s_ref),
                       Y_ref = track->eval_Y(s_ref);
            viz_msg.markers[1].points[i].x = X_ref;
            viz_msg.markers[1].points[i].y = Y_ref;
            viz_msg.markers[1].points[i].z = 0.05;

            // predicted trajectory
            const auto [X, Y, _] =
                track->frenet_to_cartesian(FrenetPose{s + x_opt(0, i), x_opt(1, i), x_opt(2, i)})
                    .first;
            viz_msg.markers[0].points[i].x = X;
            viz_msg.markers[0].points[i].y = Y;
            viz_msg.markers[0].points[i].z = 0.1;

            // TODO: add boundaries
            const auto phi_ref = track->eval_phi(s_ref);
            const auto deviation =
                track->eval_width(s_ref) -
                (car_width + this->get_parameter("car_width_inflation").as_double()) / 2;
            viz_msg.markers[2].points[i].x = X_ref - deviation * sin(phi_ref);
            viz_msg.markers[2].points[i].y = Y_ref + deviation * cos(phi_ref);
            viz_msg.markers[3].points[i].x = X_ref + deviation * sin(phi_ref);
            viz_msg.markers[3].points[i].y = Y_ref - deviation * cos(phi_ref);
        }
    }

    DiagnosticArray diag_msg;
    Publisher<DiagnosticArray>::SharedPtr diagnostics_pub;

    void create_diag_msg() {
        this->diag_msg.status.resize(1);
        this->diag_msg.status[0].name = "high_level_controller";
        this->diag_msg.status[0].values.resize(1);
        this->diag_msg.status[0].values[0].key = "runtime (ms)";
        // TODO: add second status for LLC
    }

    void update_diag_msg(const double runtime_ms) {
        this->diag_msg.header.stamp = this->now();
        // TODO: report internal status here (nominal, sending last prediction, crashed)
        this->diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        this->diag_msg.status[0].message = "nominal";
        this->diag_msg.status[0].values[0].value = to_string(runtime_ms);
    }

    ControllerDebugInfo debug_info_msg;
    Publisher<ControllerDebugInfo>::SharedPtr debug_info_pub;

    void create_debug_info_msg() {
        const auto Nf = this->controller->horizon_size();
        this->debug_info_msg.s_ref.resize(Nf + 1);
        this->debug_info_msg.n_ref.resize(Nf + 1);
        this->debug_info_msg.psi_ref.resize(Nf + 1);
        this->debug_info_msg.v_ref.resize(Nf + 1);
        this->debug_info_msg.delta_ref.resize(Nf);
        this->debug_info_msg.tau_ref.resize(Nf);
        this->debug_info_msg.s_pred.resize(Nf + 1);
        this->debug_info_msg.n_pred.resize(Nf + 1);
        this->debug_info_msg.psi_pred.resize(Nf + 1);
        this->debug_info_msg.v_pred.resize(Nf + 1);
        this->debug_info_msg.delta_pred.resize(Nf);
        this->debug_info_msg.tau_pred.resize(Nf);
    }

    void update_debug_info_msg(const Controller::StateHorizonMatrix &x_ref,
                               const Controller::ControlHorizonMatrix &u_ref,
                               const Controller::StateHorizonMatrix &x_pred,
                               const Controller::ControlHorizonMatrix &u_pred) {
        this->debug_info_msg.header.stamp = this->now();
        const auto Nf = this->controller->horizon_size();
        for (size_t i = 0; i < Nf + 1; ++i) {
            this->debug_info_msg.s_ref[i] = x_ref(0, i);
            this->debug_info_msg.n_ref[i] = x_ref(1, i);
            this->debug_info_msg.psi_ref[i] = x_ref(2, i);
            this->debug_info_msg.v_ref[i] = x_ref(3, i);
            this->debug_info_msg.s_pred[i] = x_pred(0, i);
            this->debug_info_msg.n_pred[i] = x_pred(1, i);
            this->debug_info_msg.psi_pred[i] = x_pred(2, i);
            this->debug_info_msg.v_pred[i] = x_pred(3, i);
            if (i < Nf) {
                this->debug_info_msg.delta_ref[i] = u_ref(0, i);
                this->debug_info_msg.tau_ref[i] = u_ref(1, i);
                this->debug_info_msg.delta_pred[i] = u_pred(0, i);
                this->debug_info_msg.tau_pred[i] = u_pred(1, i);
            }
        }
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
