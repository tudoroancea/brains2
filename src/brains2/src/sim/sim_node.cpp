// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "Eigen/Dense"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "brains2/common/cone_color.hpp"
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/msg/acceleration.hpp"
#include "brains2/msg/controls.hpp"
#include "brains2/msg/pose.hpp"
#include "brains2/msg/velocity.hpp"
#include "brains2/sim/sim.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nlohmann/json.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <fstream>
#include <unordered_map>

using namespace std;
using namespace brains2::msg;
using rclcpp::Publisher;
using rclcpp::Subscription;

// used for the lap time computation
bool ccw(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c) {
  return (c(1) - a(1)) * (b(0) - a(0)) > (b(1) - a(1)) * (c(0) - a(0));
}
bool intersect(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c,
               Eigen::Vector2d d) {
  return ccw(a, c, d) != ccw(b, c, d) && ccw(a, b, c) != ccw(a, b, d);
}

template <typename Derived>
Eigen::PermutationMatrix<Eigen::Dynamic>
argsort(const Eigen::DenseBase<Derived> &vec) {
  Eigen::PermutationMatrix<Eigen::Dynamic> perm(vec.size());
  std::iota(perm.indices().data(),
            perm.indices().data() + perm.indices().size(), 0);
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
  rclcpp::Subscription<Controls>::SharedPtr controls_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      alternative_controls_sub;

  // services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_cones_srv;

  // simulation variables
  double dt;
  rclcpp::TimerBase::SharedPtr sim_timer;
  brains2::sim::Sim sim;
  brains2::sim::Sim::State current_state;
  Pose pose_msg;
  Velocity velocity_msg;
  Acceleration acceleration_msg;
  Controls current_controls_msg;
  geometry_msgs::msg::TransformStamped transform;

  // cones
  visualization_msgs::msg::MarkerArray cones_marker_array;
  std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d> cones_map;

  // lap timing
  Eigen::Vector2d last_pos, start_line_pos_1, start_line_pos_2;
  double last_lap_time = 0.0, best_lap_time = 0.0;
  rclcpp::Time last_lap_time_stamp = rclcpp::Time(0, 0);

  void controls_callback(const brains2::msg::Controls::SharedPtr msg) {
    if (!this->get_parameter("manual_control").as_bool()) {
      double tau_max = this->get_parameter("tau_max").as_double(),
             delta_max = this->get_parameter("delta_max").as_double(),
             ddelta_max(0.01 * 2 * delta_max / 1.0);

      current_state.tau_FL =
          brains2::common::clip(msg->tau_fl, -tau_max, tau_max);
      current_state.tau_FR =
          brains2::common::clip(msg->tau_fr, -tau_max, tau_max);
      current_state.tau_RL =
          brains2::common::clip(msg->tau_rl, -tau_max, tau_max);
      current_state.tau_RR =
          brains2::common::clip(msg->tau_rr, -tau_max, tau_max);
      current_state.delta = brains2::common::clip(
          brains2::common::clip(msg->delta, current_state.delta - ddelta_max,
                                current_state.delta + ddelta_max),
          -delta_max, delta_max);
    }
  }

  void alternative_controls_callback(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (this->get_parameter("manual_control").as_bool()) {
      double tau_max = this->get_parameter("tau_max").as_double(),
             delta_max = this->get_parameter("delta_max").as_double(),
             ddelta_max(this->dt * 2 * delta_max / 1.0);

      current_state.tau_FL =
          brains2::common::clip(msg->linear.x * 0.25, -tau_max, tau_max);
      current_state.tau_FR =
          brains2::common::clip(msg->linear.x * 0.25, -tau_max, tau_max);
      current_state.tau_RL =
          brains2::common::clip(msg->linear.x * 0.25, -tau_max, tau_max);
      current_state.tau_RR =
          brains2::common::clip(msg->linear.x * 0.25, -tau_max, tau_max);
      current_state.delta = brains2::common::clip(
          brains2::common::clip(msg->angular.z,
                                current_state.delta - ddelta_max,
                                current_state.delta + ddelta_max),
          -delta_max, delta_max);
    }
  }

  void publish_cones_srv_cb(
      [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr request,
      [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr response) {
    this->viz_pub->publish(cones_marker_array);
  }

  void reset_srv_cb(
      [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr request,
      [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr response) {
    this->sim.reset();
    // reset lap timing
    this->last_lap_time_stamp = rclcpp::Time(0, 0);
  }

  void sim_timer_cb() {
    auto start = this->now();
    // depending on the last velocity v=sqrt(v_x^2+v_y^2), decide which model to
    // use and set its inputs
    bool use_kin6(std::hypot(x[3], x[4]) <
                  this->get_parameter("v_dyn").as_double());
    try {
      if (use_kin6) {
        sim_in_set(kin6_sim_config, kin6_sim_dims, kin6_sim_in, "x", this->x);
        sim_in_set(kin6_sim_config, kin6_sim_dims, kin6_sim_in, "u", this->u);
      } else {
        sim_in_set(dyn6_sim_config, dyn6_sim_dims, dyn6_sim_in, "x", this->x);
        sim_in_set(dyn6_sim_config, dyn6_sim_dims, dyn6_sim_in, "u", this->u);
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Caught exception: %s", e.what());
    }

    // call sim solver

    // prohibit the car from going backwards
    if (x[3] < 0.0 or (x[nx - 2] <= 0.1 and x[3] < 0.01)) {
      x[3] = 0.0;
      x[4] = 0.0;
      x[5] = 0.0;
    }

    auto end = this->now();

    // check if we have completed a lap
    Eigen::Vector2d pos(x[0], x[1]);
    if (x[3] > 0.0 and intersect(this->start_line_pos_1, this->start_line_pos_2,
                                 this->last_pos, pos)) {
      if (this->last_lap_time_stamp.nanoseconds() > 0) {
        double lap_time((end - this->last_lap_time_stamp).seconds());
        if (this->best_lap_time == 0.0 or lap_time < this->best_lap_time) {
          this->best_lap_time = lap_time;
        }
        this->last_lap_time = lap_time;
        RCLCPP_INFO(this->get_logger(),
                    "Lap completed time: %.3f s (best: %.3f s)", lap_time,
                    this->best_lap_time);
      }
      this->last_lap_time_stamp = end;
    }
    this->last_pos = pos;
    // override the pose and velocity and controls with the simulation output
    this->state_msg.header.stamp = this->now();
    this->state_msg.header.frame_id = "world";
    this->state_msg.pose.position.x = x[0];
    this->state_msg.pose.position.y = x[1];
    this->state_msg.pose.orientation = rpy_to_quaternion(0.0, 0.0, x[2]);
    this->state_msg.twist.linear.x = x[3];
    this->state_msg.twist.linear.y = x[4];
    this->state_msg.twist.angular.z = x[5];
    this->state_msg.controls.throttle = x[nx - 2];
    this->state_msg.controls.steering = x[nx - 1];
    this->state_pub->publish(this->state_msg);

    this->transform.header.stamp = this->now();
    this->transform.header.frame_id = "world";
    this->transform.child_frame_id = "car";
    this->transform.transform.translation.x = x[0];
    this->transform.transform.translation.y = x[1];
    this->transform.transform.rotation = this->state_msg.pose.orientation;
    this->tf_broadcaster->sendTransform(this->transform);

    visualization_msgs::msg::MarkerArray markers_msg;
    std::vector<visualization_msgs::msg::Marker> car_markers =
        get_car_markers();
    for (const auto &marker : car_markers) {
      markers_msg.markers.push_back(marker);
    }
    this->viz_pub->publish(markers_msg);

    // publish diagnostics
    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->now();
    diag_msg.status.resize(1);
    diag_msg.status[0].name = "sim";
    diag_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_msg.status[0].message = "OK";

    diagnostic_msgs::msg::KeyValue sim_runtime_kv;
    sim_runtime_kv.key = "sim runtime (ms)";
    sim_runtime_kv.value = std::to_string(1000 * (end - start).seconds());
    diag_msg.status[0].values.push_back(sim_runtime_kv);

    diagnostic_msgs::msg::KeyValue track_name_kv;
    track_name_kv.key = "track name";
    track_name_kv.value = this->get_parameter("track_name_or_file").as_string();
    diag_msg.status[0].values.push_back(track_name_kv);

    diagnostic_msgs::msg::KeyValue model_kv;
    model_kv.key = "model";
    model_kv.value = use_kin6 ? "kin6" : "dyn6";
    diag_msg.status[0].values.push_back(model_kv);

    diagnostic_msgs::msg::KeyValue last_lap_time_kv;
    last_lap_time_kv.key = "last lap time (s)";
    last_lap_time_kv.value = std::to_string(this->last_lap_time);
    diag_msg.status[0].values.push_back(last_lap_time_kv);

    diagnostic_msgs::msg::KeyValue best_lap_time_kv;
    best_lap_time_kv.key = "best lap time (s)";
    best_lap_time_kv.value = std::to_string(this->best_lap_time);
    diag_msg.status[0].values.push_back(best_lap_time_kv);

    this->diagnostics_pub->publish(diag_msg);
  }

  void filter_cones(Eigen::MatrixX2d cones, double X, double Y, double phi,
                    Eigen::VectorXd &rho, Eigen::VectorXd &theta) {
    // get bearing and range limits
    std::vector<double> range_limits(
        this->get_parameter("range_limits").as_double_array()),
        bearing_limits(this->get_parameter("bearing_limits").as_double_array());
    // compute the postions of the cones relative to the car
    Eigen::Vector2d pos(X, Y);
    Eigen::MatrixX2d cartesian = cones.rowwise() - pos.transpose();
    Eigen::Matrix2d rot;
    rot << std::cos(phi), -std::sin(phi), std::sin(phi), std::cos(phi);
    cartesian = cartesian * rot.transpose();
    Eigen::MatrixX2d polar(cartesian.rows(), 2);
    for (Eigen::Index i(0); i < cartesian.rows(); ++i) {
      polar(i, 0) = cartesian.row(i).norm();
      polar(i, 1) = std::atan2(cartesian(i, 1), cartesian(i, 0));
    }
    // only keep the cones that are in the range and bearing limits
    Eigen::Array<bool, Eigen::Dynamic, 1> mask =
        (range_limits[0] <= polar.col(0).array()) &&
        (polar.col(0).array() <= range_limits[1]) &&
        (bearing_limits[0] <= polar.col(1).array()) &&
        (polar.col(1).array() <= bearing_limits[1]);
    size_t n_cones(mask.count());
    rho.conservativeResize(n_cones);
    theta.conservativeResize(n_cones);
    for (Eigen::Index i(0), j(0); i < cones.rows(); ++i) {
      if (mask(i)) {
        rho(j) = polar(i, 0);
        theta(j) = polar(i, 1);
        ++j;
      }
    }
  }

  void create_cones_markers(const std::string &track_name_or_file) {
    cones_map = load_cones(track_name_or_file);

    // set deleteall to all the markers in the cones_marker_array and publish it
    for (auto &marker : cones_marker_array.markers) {
      marker.action = visualization_msgs::msg::Marker::DELETEALL;
    }
    this->viz_pub->publish(cones_marker_array);
    // create new cones
    cones_marker_array.markers.clear();
    for (auto &[color, cones] : cones_map) {
      for (int i = 0; i < cones.rows(); i++) {
        cones_marker_array.markers.push_back(this->get_cone_marker(
            i, cones(i, 0), cones(i, 1),
            is_orange(color) ? "orange" : cone_color_to_string(color),
            color != ConeColor::BIG_ORANGE));
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %lu cones from %s",
                cones_marker_array.markers.size(), track_name_or_file.c_str());
    // if there are big orange cones, find the ones that have the smallest y
    // coordinate and set them as start line
    if (cones_map.find(ConeColor::BIG_ORANGE) != cones_map.end()) {
      Eigen::MatrixX2d big_orange_cones(cones_map[ConeColor::BIG_ORANGE]);
      if (big_orange_cones.rows() >= 2) {
        // Find the indices that would sort the second column
        Eigen::PermutationMatrix<Eigen::Dynamic> indices =
            argsort(big_orange_cones.col(1));
        // Extract the corresponding rows
        start_line_pos_1 = big_orange_cones.row(indices.indices()(0));
        start_line_pos_2 = big_orange_cones.row(indices.indices()(1));
        RCLCPP_INFO(this->get_logger(),
                    "Found start line at (%.3f, %.3f) and (%.3f, %.3f)",
                    start_line_pos_1(0), start_line_pos_1(1),
                    start_line_pos_2(0), start_line_pos_2(1));
      }
    }
  }

  visualization_msgs::msg::Marker get_cone_marker(uint64_t id, double X,
                                                  double Y, std::string color,
                                                  bool small) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = color + "_cones";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource =
        "file://" + ament_index_cpp::get_package_share_directory("ihm2") +
        "/meshes/cone.stl";
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.position.x = X;
    marker.pose.position.y = Y;
    marker.pose.orientation = rpy_to_quaternion(0.0, 0.0, 0.0);
    if (!small) {
      marker.scale.z *= (285.0 / 228.0);
      marker.scale.x *= (285.0 / 228.0);
      marker.scale.y *= (505.0 / 325.0);
    }
    marker.color = marker_colors(color);
    return marker;
  }

  std::vector<visualization_msgs::msg::Marker> get_car_markers() {
    double X = x[0], Y = x[1], phi = x[2], delta = x[nx - 1];
    std::vector<visualization_msgs::msg::Marker> markers;
    if (!this->is_wsl || this->has_internet) {
      // we either are not on WSL (i.e. we are on Linux of macOS) or
      // we are and we have internet connection, so we use mesh
      // markers for the car note: we used lazey evaluation to avoid
      // calling is_internet_connected() if we are not on WSL
      std::string car_mesh = this->get_parameter("car_mesh").as_string();
      markers.resize(car_mesh == "lego-lrt4.stl" ? 1 : 5);
      markers[0].header.frame_id = "world";
      markers[0].ns = "car";
      markers[0].type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      markers[0].action = visualization_msgs::msg::Marker::MODIFY;
      markers[0].mesh_resource = this->is_wsl
                                     ? this->get_remote_mesh_path(car_mesh)
                                     : this->get_local_mesh_path(car_mesh);
      markers[0].pose.position.x = X;
      markers[0].pose.position.y = Y;
      markers[0].pose.orientation = rpy_to_quaternion(0.0, 0.0, phi);
      markers[0].scale.x = 1.0;
      markers[0].scale.y = 1.0;
      markers[0].scale.z = 1.0;
      markers[0].color = marker_colors("white");
      for (size_t i(1); i < 5; ++i) {
        markers[i].header.frame_id = "world";
        markers[i].ns = "car";
        markers[i].id = i;
        markers[i].type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        markers[i].mesh_resource =
            this->is_wsl ? this->get_remote_mesh_path("ariane_wheel.stl")
                         : this->get_local_mesh_path("ariane_wheel.stl");
        markers[i].action = visualization_msgs::msg::Marker::MODIFY;
        double wheel_x(i < 3 ? 0.7853 : -0.7853),
            wheel_y(i % 2 == 0 ? 0.6291 : -0.6291),
            wheel_phi(i < 3 ? delta : 0.0);
        // the STL represents right wheels, so for left wheels (i.e.
        // i=1,3) we add a yaw of pi
        wheel_phi += (i % 2 == 1 ? 0.0 : M_PI);
        markers[i].pose.position.x =
            X + wheel_x * std::cos(phi) - wheel_y * std::sin(phi);
        markers[i].pose.position.y =
            Y + wheel_x * std::sin(phi) + wheel_y * std::cos(phi);
        markers[i].pose.position.z = 0.20809;
        markers[i].pose.orientation =
            rpy_to_quaternion(0.0, 0.0, phi + wheel_phi);
        markers[i].scale.x = 1.0;
        markers[i].scale.y = 1.0;
        markers[i].scale.z = 1.0;
        markers[i].color = marker_colors("black");
      }
    } else {
      // we are on WSL and we don't have internet connection, so we
      // use cubes and cylinders for the car
      markers.resize(5);
      markers[0].header.frame_id = "world";
      markers[0].ns = "car";
      markers[0].id = 0;
      markers[0].type = visualization_msgs::msg::Marker::CUBE;
      markers[0].action = visualization_msgs::msg::Marker::MODIFY;
      markers[0].pose.position.x = X + 0.5 * std::cos(phi);
      markers[0].pose.position.y = Y + 0.5 * std::sin(phi);
      markers[0].pose.position.z = 0.6 / 2.0;
      markers[0].pose.orientation = rpy_to_quaternion(0.0, 0.0, phi);
      markers[0].scale.x = 3.19;
      markers[0].scale.y = 1.05;
      markers[0].scale.z = 0.6;
      markers[0].color = marker_colors("white");
      for (size_t i(1); i < 5; ++i) {
        markers[i].header.frame_id = "world";
        markers[i].ns = "car";
        markers[i].id = i;
        markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
        markers[i].action = visualization_msgs::msg::Marker::MODIFY;
        double wheel_x(i < 3 ? 0.7853 : -0.7853),
            wheel_y(i % 2 == 0 ? 0.6291 : -0.6291),
            wheel_phi(i < 3 ? delta : 0.0);
        markers[i].pose.position.x =
            X + wheel_x * std::cos(phi) - wheel_y * std::sin(phi);
        markers[i].pose.position.y =
            Y + wheel_x * std::sin(phi) + wheel_y * std::cos(phi);
        markers[i].pose.position.z = 0.232;
        markers[i].pose.orientation =
            rpy_to_quaternion(M_PI_2, 0.0, phi + wheel_phi);
        markers[i].scale.x = 0.41618;
        markers[i].scale.y = 0.41618;
        markers[i].scale.z = 0.21;
        markers[i].color = marker_colors("black");
      }
    }
    return markers;
  }

  inline std::string get_local_mesh_path(std::string mesh_file) {
    return "file://" + ament_index_cpp::get_package_share_directory("ihm2") +
           "/meshes/" + mesh_file;
  }
  inline std::string get_remote_mesh_path(std::string mesh_file) {
    return "https://raw.github.com/EPFL-RT-Driverless/resources/main/meshes/" +
           mesh_file;
  }

public:
  SimNode() : Node("sim_node") {
    this->declare_parameter<double>("freq", 100.0);
    this->declare_parameter<bool>("manual_control", true);
    this->declare_parameter<std::string>("track_name_or_file",
                                         "fsds_competition_2");
    this->declare_parameter<bool>("use_meshes", true);
    this->declare_parameter<std::string>("car_mesh", "ariane.stl");
    this->declare_parameter<double>("tau_max", 276.75);
    this->declare_parameter<double>("delta_max", 0.5);
    this->declare_parameter<double>("v_dyn", 1.0);
    this->declare_parameter<std::vector<double>>("range_limits", {0.0, 15.0});
    this->declare_parameter<std::vector<double>>(
        "bearing_limits", {-deg2rad(50.0), deg2rad(50.0)});

    // initialize x and u with zeros
    this->reset_srv_cb(nullptr, nullptr);

    // publishers
    this->viz_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/brains2/viz/sim", 10);
    this->pose_pub = this->create_publisher<Pose>("/brains2/pose", 10);
    this->velocity_pub =
        this->create_publisher<Velocity>("/brains2/velocity", 10);
    this->acceleration_pub =
        this->create_publisher<Acceleration>("/brains2/acceleration", 10);
    this->current_controls_pub =
        this->create_publisher<Controls>("/brains2/controls", 10);
    this->diagnostics_pub =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/brains2/diagnostics", 10);
    this->tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // subscribers
    this->controls_sub = this->create_subscription<brains2::msg::Controls>(
        "/brains2/controls", 10,
        std::bind(&SimNode::controls_callback, this, std::placeholders::_1));
    this->alternative_controls_sub =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/brains2/alternative_controls", 10,
            std::bind(&SimNode::alternative_controls_callback, this,
                      std::placeholders::_1));

    // services
    this->reset_srv = this->create_service<std_srvs::srv::Empty>(
        "/brains2/reset",
        std::bind(&SimNode::reset_srv_cb, this, std::placeholders::_1,
                  std::placeholders::_2));
    this->publish_cones_srv = this->create_service<std_srvs::srv::Empty>(
        "/brains2/publish_cones_markers",
        std::bind(&SimNode::publish_cones_srv_cb, this, std::placeholders::_1,
                  std::placeholders::_2));

    // load cones from track file and create the markers for the cones
    this->create_cones_markers(
        this->get_parameter("track_name_or_file").as_string());

    // call once the the publish cones service
    this->publish_cones_srv_cb(nullptr, nullptr);

    // create a timer for the simulation loop (one simulation step and
    // publishing the car mesh)
    dt = 1 / this->get_parameter("freq").as_double();
    this->sim_timer =
        this->create_wall_timer(std::chrono::duration<double>(dt),
                                std::bind(&SimNode::sim_timer_cb, this));
  }

  ~SimNode() {
    int status = ihm2_kin6_acados_sim_free(
        (ihm2_kin6_sim_solver_capsule *)kin6_sim_capsule);
    if (status) {
      RCLCPP_WARN(this->get_logger(),
                  "ihm2_kin6_acados_sim_free() returned status %d.", status);
    }
    status = ihm2_dyn6_acados_sim_free(
        (ihm2_dyn6_sim_solver_capsule *)dyn6_sim_capsule);
    if (status) {
      RCLCPP_WARN(this->get_logger(),
                  "ihm2_dyn6_acados_sim_free() returned status %d.", status);
    }
    ihm2_kin6_acados_sim_solver_free_capsule(
        (ihm2_kin6_sim_solver_capsule *)kin6_sim_capsule);
    ihm2_dyn6_acados_sim_solver_free_capsule(
        (ihm2_dyn6_sim_solver_capsule *)dyn6_sim_capsule);
    free(x);
    free(u);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimNode>();
  try {
    rclcpp::spin(node);
  } catch (const FatalNodeError &e) {
    RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Caught unknown exception");
  }
  rclcpp::shutdown();
  return 0;
}
