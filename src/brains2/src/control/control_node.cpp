// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include <memory>
#include <rclcpp/logging.hpp>
#include <tuple>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "brains2/common/cone_color.hpp"
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/control/controller.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/external/optional.hpp"
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
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace brains2::msg;
using rclcpp::Publisher;
using rclcpp::Subscription;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        const auto dt = 1 / this->declare_parameter("freq", 20.0);
        const auto Nf = this->declare_parameter("Nf", 10);

        // Limits
        const brains2::control::Controller::Limits limits{this->declare_parameter("v_x_max", 10.0),
                                                          this->declare_parameter("delta_max", 0.5),
                                                          this->declare_parameter("tau_max", 1.0)};

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

        {
            std::string init_message{};
            IC_CONFIG.prefix("");
            IC_CONFIG.output(init_message);
            IC(model_params, limits);
            RCLCPP_INFO(this->get_logger(), "Loaded model parameters: %s", init_message.c_str());
        }

#else
#error CAR_CONSTANTS_PATH is not defined
#endif

        // cost params
        const brains2::control::Controller::CostParams cost_params{
            .v_ref = this->declare_parameter("v_x_ref", 5.0),
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
        this->controller = std::make_unique<brains2::control::Controller>(static_cast<size_t>(Nf),
                                                                          model_params,
                                                                          limits,
                                                                          cost_params);

        // Publishers and subscribers
        this->target_controls_pub =
            this->create_publisher<Controls>("/brains2/target_controls", 10);
        this->viz_pub =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/brains2/viz/control",
                                                                         10);
        this->diagnostics_pub =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/brains2/diagnostics",
                                                                          10);
    }

private:
    // publishers
    Publisher<Controls>::SharedPtr target_controls_pub;
    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;

    // subscribers
    Subscription<Pose>::SharedPtr pose_sub;
    Subscription<Velocity>::SharedPtr velocity_sub;
    Subscription<Controls>::SharedPtr current_controls_sub;
    Subscription<TrackEstimate>::SharedPtr track_estimate_sub;

    // controller
    brains2::control::Controller::State state;
    std::unique_ptr<brains2::control::Controller> controller;

    void pose_cb(const Pose::SharedPtr msg) {
        state.X = msg->x;
        state.Y = msg->y;
        state.phi = msg->phi;
    }

    void vel_cb(const Velocity::SharedPtr msg) {
        state.v = std::hypot(msg->v_x, msg->v_y);
    }

    void current_controls_cb([[maybe_unused]] const Controls::SharedPtr msg) {
    }

    void track_estimate_cb(const TrackEstimate::SharedPtr msg) {
        const auto track = brains2::common::Track::from_values(msg->s_cen,
                                                               msg->x_cen,
                                                               msg->y_cen,
                                                               msg->phi_cen,
                                                               msg->kappa_cen,
                                                               msg->w_cen);
        if (!track) {
            RCLCPP_ERROR(this->get_logger(),
                         "Could not construct Track object from received message (arrays with "
                         "different lengths ?)");
        }

        const auto controls = this->controller->compute_control(this->state, *track);
        if (!controls.has_value()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Error in MPC solver: %s",
                         brains2::control::Controller::to_string(controls.error()).c_str());
        }
        // Publish controls
        brains2::msg::Controls controls_msg;
        controls_msg.tau_fl = controls->tau / 4;
        controls_msg.tau_fr = controls->tau / 4;
        controls_msg.tau_rl = controls->tau / 4;
        controls_msg.tau_rr = controls->tau / 4;
        controls_msg.delta = controls->delta;
        this->target_controls_pub->publish(controls_msg);

        // TODO: add viz
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
