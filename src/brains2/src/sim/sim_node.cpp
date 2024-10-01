// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include <tuple>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "brains2/common/cone_color.hpp"
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/external/optional.hpp"
#include "brains2/msg/acceleration.hpp"
#include "brains2/msg/controls.hpp"
#include "brains2/msg/pose.hpp"
#include "brains2/msg/velocity.hpp"
#include "brains2/sim/sim.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace brains2::msg;
using rclcpp::Publisher;
using rclcpp::Subscription;

// used for the lap time computation
static bool ccw(std::pair<double, double> a,
                std::pair<double, double> b,
                std::pair<double, double> c) {
    return (c.second - a.second) * (b.first - a.first) >
           (b.second - a.second) * (c.first - a.first);
}
static bool intersect(std::pair<double, double> a,
                      std::pair<double, double> b,
                      std::pair<double, double> c,
                      std::pair<double, double> d) {
    return ccw(a, c, d) != ccw(b, c, d) && ccw(a, b, c) != ccw(a, b, d);
}

template <typename Derived>
static Eigen::PermutationMatrix<Eigen::Dynamic> argsort(const Eigen::DenseBase<Derived> &vec) {
    Eigen::PermutationMatrix<Eigen::Dynamic> perm(vec.size());
    std::iota(perm.indices().data(), perm.indices().data() + perm.indices().size(), 0);
    std::sort(perm.indices().data(),
              perm.indices().data() + perm.indices().size(),
              [&](int i, int j) { return vec(i) < vec(j); });
    return perm;
}

// actual simulation node
class SimNode : public rclcpp::Node {
private:
    // publishers
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    Publisher<Pose>::SharedPtr pose_pub;
    Publisher<Velocity>::SharedPtr velocity_pub;
    Publisher<Acceleration>::SharedPtr acceleration_pub;
    Publisher<Controls>::SharedPtr current_controls_pub;
    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;

    // subscribers
    rclcpp::Subscription<Controls>::SharedPtr target_controls_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr alternative_target_controls_sub;

    // services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_cones_srv;

    // simulation variables
    double dt;
    rclcpp::TimerBase::SharedPtr sim_timer;
    std::unique_ptr<brains2::sim::Sim> sim;
    brains2::sim::Sim::State state;
    brains2::sim::Sim::Control control;
    brains2::sim::Sim::Accels accels;

    // messages to publish
    Pose pose_msg;
    Velocity velocity_msg;
    Acceleration acceleration_msg;
    Controls current_controls_msg;
    geometry_msgs::msg::TransformStamped transform;
    diagnostic_msgs::msg::DiagnosticArray diag_msg;

    // cones
    visualization_msgs::msg::MarkerArray cones_marker_array;
    visualization_msgs::msg::MarkerArray car_markers_msg;

    // lap timing
    std::pair<double, double> last_position, start_line_pos_1, start_line_pos_2;
    double last_lap_time = 0.0, best_lap_time = 0.0;
    rclcpp::Time last_lap_time_stamp = rclcpp::Time(0, 0);

    void controls_callback(const brains2::msg::Controls::SharedPtr msg) {
        if (!this->get_parameter("manual_control").as_bool()) {
            control.u_tau_FL = msg->tau_fl;
            control.u_tau_FR = msg->tau_fr;
            control.u_tau_RL = msg->tau_rl;
            control.u_tau_RR = msg->tau_rr;
            control.u_delta = msg->delta;
        }
    }

    void alternative_controls_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (this->get_parameter("manual_control").as_bool()) {
            control.u_tau_FL = msg->linear.x;
            control.u_tau_FR = msg->linear.x;
            control.u_tau_RL = msg->linear.x;
            control.u_tau_RR = msg->linear.x;
            control.u_delta = msg->angular.z;
        }
    }

    void publish_cones_srv_cb(
        [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr request,
        [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr response) {
        this->viz_pub->publish(cones_marker_array);
    }

    void reset_srv_cb([[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr request,
                      [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr response) {
        // Reset the state and accelerations
        this->state =
            brains2::sim::Sim::State{0.0, 0.0, M_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        this->accels = brains2::sim::Sim::Accels{0.0, 0.0};
        // Reset lap timing
        this->last_lap_time_stamp = rclcpp::Time(0, 0);
    }

    void sim_timer_cb() {
        auto start = this->now();

        // call sim solver
        brains2::sim::Sim::State new_state{};
        auto expected_sim_result = sim->simulate(state, control, dt);
        if (!expected_sim_result) {
            throw std::runtime_error("Simulation error: " +
                                     brains2::sim::Sim::to_string(expected_sim_result.error()));
        }
        std::tie(new_state, accels) = expected_sim_result.value();
        state = new_state;

        auto end = this->now();

        // check if we have completed a lap
        std::pair<double, double> pos{state.X, state.Y};
        if (state.v_x > 0.0 &&
            intersect(this->start_line_pos_1, this->start_line_pos_2, this->last_position, pos)) {
            if (this->last_lap_time_stamp.nanoseconds() > 0) {
                double lap_time((end - this->last_lap_time_stamp).seconds());
                if (this->best_lap_time == 0.0 or lap_time < this->best_lap_time) {
                    this->best_lap_time = lap_time;
                }
                this->last_lap_time = lap_time;
                RCLCPP_INFO(this->get_logger(),
                            "Lap completed time: %.3f s (best: %.3f s)",
                            lap_time,
                            this->best_lap_time);
            }
            this->last_lap_time_stamp = end;
        }
        this->last_position = pos;

        // Publish pose
        this->pose_msg.header.stamp = this->now();
        this->pose_msg.x = state.X;
        this->pose_msg.y = state.Y;
        this->pose_msg.phi = state.phi;
        this->pose_pub->publish(this->pose_msg);

        // Publish velocity
        this->velocity_msg.header.stamp = this->now();
        this->velocity_msg.v_x = state.v_x;
        this->velocity_msg.v_y = state.v_y;
        this->velocity_msg.omega = state.omega;
        this->velocity_pub->publish(this->velocity_msg);

        // Publish acceleration
        this->acceleration_msg.header.stamp = this->now();
        this->acceleration_msg.a_x = accels.a_x;
        this->acceleration_msg.a_y = accels.a_y;
        this->acceleration_pub->publish(this->acceleration_msg);

        // Publish current controls
        this->current_controls_msg.header.stamp = this->now();
        this->current_controls_msg.delta = state.delta;
        this->current_controls_msg.tau_fl = state.tau_FL;
        this->current_controls_msg.tau_fr = state.tau_FR;
        this->current_controls_msg.tau_rl = state.tau_RL;
        this->current_controls_msg.tau_rr = state.tau_RR;
        this->current_controls_pub->publish(this->current_controls_msg);

        // Publish transform from world to car
        this->transform.header.stamp = this->now();
        this->transform.header.frame_id = "world";
        this->transform.child_frame_id = "car";
        this->transform.transform.translation.x = state.X;
        this->transform.transform.translation.y = state.Y;
        this->transform.transform.rotation =
            brains2::common::rpy_to_quaternion_msg(0.0, 0.0, state.phi);
        this->tf_broadcaster->sendTransform(this->transform);

        // publish diagnostics
        diag_msg.status[0].values[0].value = std::to_string(1000 * (end - start).seconds());
        diag_msg.status[0].values[1].value = this->get_parameter("track_name").as_string();
        diag_msg.status[0].values[2].value = std::to_string(this->last_lap_time);
        diag_msg.status[0].values[3].value = std::to_string(this->best_lap_time);
        this->diagnostics_pub->publish(diag_msg);

        // visualization_msgs::msg::MarkerArray markers_msg;
        this->update_tire_markers();
        this->viz_pub->publish(car_markers_msg);
    }

    void create_diagnostics_message() {
        diag_msg.header.stamp = this->now();
        diag_msg.status.resize(1);
        diag_msg.status[0].name = "sim";
        diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diag_msg.status[0].message = "OK";
        diag_msg.status[0].values.resize(5);
        diag_msg.status[0].values[0].key = "sim runtime (ms)";
        diag_msg.status[0].values[1].key = "track name";
        diag_msg.status[0].values[2].key = "last lap time (s)";
        diag_msg.status[0].values[3].key = "best lap time (s)";
    }

    void create_cones_markers(const std::string &track_name) {
        auto cones_map_opt = brains2::common::load_cones_from_track_database(track_name);
        if (!cones_map_opt) {
            throw std::runtime_error("Could not load cones from track database.");
        }
        const auto &cones_map = cones_map_opt.value();

        // create new cones
        cones_marker_array.markers.resize(cones_map.at(brains2::common::ConeColor::BLUE).rows() +
                                          cones_map.at(brains2::common::ConeColor::YELLOW).rows() +
                                          cones_map.at(brains2::common::ConeColor::ORANGE).rows());
        for (auto &[color, cones] : cones_map) {
            for (int i = 0; i < cones.rows(); i++) {
                cones_marker_array.markers.push_back(
                    this->get_cone_marker(i,
                                          cones(i, 0),
                                          cones(i, 1),
                                          cone_color_to_string(color),
                                          color != brains2::common::ConeColor::ORANGE));
            }
        }
        RCLCPP_INFO(this->get_logger(),
                    "Loaded %lu cones from %s",
                    cones_marker_array.markers.size(),
                    track_name.c_str());

        // TODO: may need to be reworked for tracks with a sharp corner at the start.
        // Find the orange cones that have the smallest y coordinate and set them as start line.
        if (cones_map.find(brains2::common::ConeColor::ORANGE) == cones_map.end() ||
            cones_map.at(brains2::common::ConeColor::ORANGE).rows() < 4) {
            std::cerr << cones_map.at(brains2::common::ConeColor::ORANGE).rows() << std::endl;
            throw std::runtime_error("Could not find orange cones in track database.");
        }
        Eigen::MatrixX2d orange_cones(cones_map.at(brains2::common::ConeColor::ORANGE));
        // Find the indices that would sort the second column
        Eigen::PermutationMatrix<Eigen::Dynamic> indices = argsort(orange_cones.col(1));
        // Extract the corresponding rows
        start_line_pos_1.first = orange_cones(indices.indices()(0), 0);
        start_line_pos_1.second = orange_cones(indices.indices()(0), 1);
        start_line_pos_2.first = orange_cones(indices.indices()(1), 0);
        start_line_pos_2.second = orange_cones(indices.indices()(1), 1);
        RCLCPP_INFO(this->get_logger(),
                    "Found start line at (%.3f, %.3f) and (%.3f, %.3f)",
                    start_line_pos_1.first,
                    start_line_pos_1.second,
                    start_line_pos_2.first,
                    start_line_pos_2.second);
    }

    visualization_msgs::msg::Marker get_cone_marker(
        uint64_t id, double X, double Y, std::string color, bool small) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = color + "_cones";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::MODIFY;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = this->get_local_mesh_path("cone.stl");
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.pose.position.x = X;
        marker.pose.position.y = Y;
        marker.pose.orientation = brains2::common::rpy_to_quaternion_msg(0.0, 0.0, 0.0);
        if (!small) {
            marker.scale.x *= (285.0 / 228.0);
            marker.scale.y *= (285.0 / 228.0);
            marker.scale.z *= (505.0 / 325.0);
        }
        marker.color = brains2::common::marker_colors(color);
        return marker;
    }

    void create_car_markers() {
        std::vector<visualization_msgs::msg::Marker> &markers = car_markers_msg.markers;
        markers.resize(5);
        markers[0].header.frame_id = "car";
        markers[0].ns = "chassis";
        markers[0].type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        markers[0].action = visualization_msgs::msg::Marker::MODIFY;
        markers[0].mesh_resource = this->get_local_mesh_path("ariane.stl");
        markers[0].pose.orientation = brains2::common::rpy_to_quaternion_msg(0.0, 0.0, 0.0);
        markers[0].scale.x = 1.0;
        markers[0].scale.y = 1.0;
        markers[0].scale.z = 1.0;
        markers[0].color = brains2::common::marker_colors("white");
        for (size_t i(1); i < 5; ++i) {
            markers[i].header.frame_id = "car";
            markers[i].ns = "tires";
            markers[i].id = i;
            markers[i].type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            markers[i].mesh_resource = this->get_local_mesh_path("ariane_wheel.stl");
            markers[i].action = visualization_msgs::msg::Marker::MODIFY;
            // The wheels are listed in CW order, starting from the front left
            markers[i].pose.position.x = (i < 3 ? 0.7853 : -0.7853);
            markers[i].pose.position.y = (i % 2 == 1 ? 0.6291 : -0.6291);
            markers[i].pose.position.z = 0.20809;
            // the STL represents left wheels, so for right wheels (i.e. i=2,4) we add a yaw of pi
            markers[i].pose.orientation =
                brains2::common::rpy_to_quaternion_msg(0.0, 0.0, i % 2 == 0 ? 0.0 : M_PI);
            markers[i].scale.x = 1.0;
            markers[i].scale.y = 1.0;
            markers[i].scale.z = 1.0;
            markers[i].color = brains2::common::marker_colors("dark_gray");
        }
    }

    void update_tire_markers() {
        car_markers_msg.markers[0].header.stamp = this->now();
        car_markers_msg.markers[1].header.stamp = this->now();
        car_markers_msg.markers[2].header.stamp = this->now();
        car_markers_msg.markers[3].header.stamp = this->now();
        car_markers_msg.markers[4].header.stamp = this->now();

        car_markers_msg.markers[1].pose.orientation =
            brains2::common::rpy_to_quaternion_msg(0.0, 0.0, state.delta);
        car_markers_msg.markers[2].pose.orientation =
            brains2::common::rpy_to_quaternion_msg(0.0, 0.0, M_PI + state.delta);
    }

    inline std::string get_local_mesh_path(std::string mesh_file) {
        return "file://" + ament_index_cpp::get_package_share_directory("brains2") + "/meshes/" +
               mesh_file;
    }

public:
    SimNode() : Node("sim_node"), sim{}, state{}, control{}, accels{} {
        // Declare all node parameters
        this->declare_parameter<double>("freq", 100.0);
        this->declare_parameter<bool>("manual_control", true);
        this->declare_parameter<std::string>("track_name", "alpha");

        // Compute sampling time
        dt = 1 / this->get_parameter("freq").as_double();

#ifdef CAR_CONSTANTS_PATH
        // load yaml file with car constants
        YAML::Node car_constants = YAML::LoadFile(CAR_CONSTANTS_PATH);
        const brains2::sim::Sim::Parameters params{
            car_constants["inertia"]["mass"].as<double>(),
            car_constants["inertia"]["yaw_inertia"].as<double>(),
            car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
            car_constants["geometry"]["wheelbase"].as<double>() -
                car_constants["geometry"]["cog_to_rear_axle"].as<double>(),
            car_constants["drivetrain"]["C_m0"].as<double>(),
            car_constants["drivetrain"]["C_r0"].as<double>(),
            car_constants["drivetrain"]["C_r1"].as<double>(),
            car_constants["drivetrain"]["C_r2"].as<double>(),
            car_constants["actuators"]["motor_time_constant"].as<double>(),
            car_constants["actuators"]["steering_time_constant"].as<double>(),
            car_constants["geometry"]["cog_height"].as<double>(),
            car_constants["geometry"]["axle_track"].as<double>(),
            car_constants["aero"]["C_downforce"].as<double>(),
            car_constants["pacejka"]["constant"]["Ba"].as<double>(),
            car_constants["pacejka"]["constant"]["Ca"].as<double>(),
            car_constants["pacejka"]["constant"]["Da"].as<double>(),
            car_constants["pacejka"]["constant"]["Ea"].as<double>()};
        const brains2::sim::Sim::Limits limits{
            car_constants["actuators"]["torque_max"].as<double>(),
            car_constants["actuators"]["steering_max"].as<double>(),
            car_constants["actuators"]["steering_rate_max"].as<double>()};

        {
            std::string init_message{};
            IC_CONFIG.prefix("");
            IC_CONFIG.output(init_message);
            IC(params, limits);
            RCLCPP_INFO(this->get_logger(),
                        "Sim initialized with parameters: %s",
                        init_message.c_str());
        }

        // Create Sim object
        this->sim = std::make_unique<brains2::sim::Sim>(params, limits);
#else
#error CAR_CONSTANTS_PATH is not defined
#endif
        // Reset the sim to initialize state and controls
        this->reset_srv_cb(nullptr, nullptr);

        // Create ros publishers.
        this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        this->pose_pub = this->create_publisher<Pose>("/brains2/pose", 10);
        this->velocity_pub = this->create_publisher<Velocity>("/brains2/velocity", 10);
        this->acceleration_pub = this->create_publisher<Acceleration>("/brains2/acceleration", 10);
        this->current_controls_pub =
            this->create_publisher<Controls>("/brains2/current_controls", 10);
        this->viz_pub =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/brains2/viz/sim", 10);
        this->diagnostics_pub =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/brains2/diagnostics",
                                                                          10);

        // Create ros subscribers.
        this->target_controls_sub = this->create_subscription<brains2::msg::Controls>(
            "/brains2/target_controls",
            10,
            std::bind(&SimNode::controls_callback, this, std::placeholders::_1));
        this->alternative_target_controls_sub =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/brains2/alternative_target_controls",
                10,
                std::bind(&SimNode::alternative_controls_callback, this, std::placeholders::_1));

        // Create ros services.
        this->reset_srv = this->create_service<std_srvs::srv::Empty>(
            "/brains2/reset",
            std::bind(&SimNode::reset_srv_cb, this, std::placeholders::_1, std::placeholders::_2));
        this->publish_cones_srv =
            this->create_service<std_srvs::srv::Empty>("/brains2/publish_cones_markers",
                                                       std::bind(&SimNode::publish_cones_srv_cb,
                                                                 this,
                                                                 std::placeholders::_1,
                                                                 std::placeholders::_2));
        // Create messages
        this->create_diagnostics_message();
        this->create_car_markers();

        // load cones from track file and create the markers for the cones
        this->create_cones_markers(this->get_parameter("track_name").as_string());

        // call once the the publish cones service
        this->publish_cones_srv_cb(nullptr, nullptr);

        // create a timer for the simulation loop (one simulation step and
        // publishing the car mesh)
        this->sim_timer = this->create_wall_timer(std::chrono::duration<double>(dt),
                                                  std::bind(&SimNode::sim_timer_cb, this));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
