#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <memory>
#include <numeric>
#include "brains2/common/marker_color.hpp"
#include "brains2/common/math.hpp"
#include "brains2/common/tracks.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/msg/fsm.hpp"
#include "brains2/msg/pose.hpp"
#include "brains2/msg/track_estimate.hpp"
#include "brains2/msg/velocity.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std;
using namespace brains2::msg;
using namespace brains2::common;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using rclcpp::Publisher;
using rclcpp::Service;
using rclcpp::Subscription;

class CoordinationNode : public rclcpp::Node {
public:
    CoordinationNode() : Node("coordination_node") {
        this->declare_parameter("laps", 1);
        this->declare_parameter("s_thresh", 7.0);

        // Initialize FSM state
        this->fsm.state = FSM::IDLE;

        // Create publishers, subscribers, services and timers
        this->fsm_pub = this->create_publisher<FSM>("/brains2/fsm", 10);
        this->diag_sub = this->create_subscription<DiagnosticArray>(
            "/brains2/diagnostics",
            10,
            [this](const DiagnosticArray::ConstSharedPtr msg) { this->diag_cb(msg); });
        this->track_estimate_sub = this->create_subscription<TrackEstimate>(
            "/brains2/track_estimate",
            10,
            [this](const TrackEstimate::ConstSharedPtr msg) { this->track_estimate_cb(msg); });
        this->vel_sub = this->create_subscription<Velocity>(
            "/brains2/velocity",
            10,
            [this](const Velocity::ConstSharedPtr msg) { this->vel_cb(msg); });
        this->pose_sub = this->create_subscription<Pose>(
            "/brains2/pose",
            10,
            [this](const Pose::ConstSharedPtr msg) { this->pose_cb(msg); });
        this->start_srv = this->create_service<std_srvs::srv::Empty>(
            "/brains2/start",
            [this](const std_srvs::srv::Empty::Request::SharedPtr req,
                   const std_srvs::srv::Empty::Response::SharedPtr res) { this->start_cb(); });
        this->timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                              [this]() { this->periodic_fsm_state_publish(); });
    }

private:
    // FSM state
    Publisher<FSM>::SharedPtr fsm_pub;
    FSM fsm;

    // Error diagnostic make the state transition to EMERGENCY_STOPPING
    Subscription<DiagnosticArray>::SharedPtr diag_sub;
    void diag_cb(const DiagnosticArray::ConstSharedPtr msg) {
        for (const auto &status : msg->status) {
            if (status.level == DiagnosticStatus::ERROR) {
                RCLCPP_ERROR(this->get_logger(),
                             "Diagnostic from %s has error \"%s\"",
                             status.name.c_str(),
                             status.message.c_str());
                if (this->fsm.state != FSM::EMERGENCY_STOPPING && this->fsm.state != FSM::STOPPED) {
                    this->fsm.header.stamp = this->now();
                    this->fsm.state = FSM::EMERGENCY_STOPPING;
                    this->fsm_pub->publish(this->fsm);
                }
            }
        }
    }

    // Receive track estimate messages
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
            // TODO: what do we do when the track estimate is invalid? do we directly go into
            // EMERGENCY_STOPPING?
            // If there is a problem with the TE node, it will probably already send an error
            // diagnostic, so we will either way transition to EMERGENCY_STOPPING.
        } else {
            this->track = std::make_unique<Track>(*track_expected);
        }
    }

    // Check for small velocities to know when we stopped
    Subscription<Velocity>::SharedPtr vel_sub;
    void vel_cb(const Velocity::ConstSharedPtr msg) {
        if ((this->fsm.state == FSM::NOMINAL_STOPPING ||
             this->fsm.state == FSM::EMERGENCY_STOPPING) &&
            std::hypot(msg->v_x, msg->v_y) < 0.01) {
            this->fsm.header.stamp = this->now();
            this->fsm.state = FSM::STOPPED;
            this->fsm_pub->publish(this->fsm);
        }
    }

    // Check if the pose made enough track progress to count a lap
    Subscription<Pose>::SharedPtr pose_sub;
    double s_proj = 0.0, last_s_proj = 0.0;
    int8_t completed_laps = -1;
    static constexpr uint64_t MIN_STEPS_SINCE_LAST_LAP = 100;
    uint64_t steps_since_last_lap = MIN_STEPS_SINCE_LAST_LAP;
    void pose_cb(const Pose::ConstSharedPtr msg) {
        // If we are in RACING state, check if we have completed the required number of laps
        if (this->fsm.state == FSM::RACING && this->track) {
            // Project pose
            const auto [s, pos] = this->track->project(msg->x, msg->y, s_proj, 10.0);
            last_s_proj = s_proj;
            s_proj = s;
            // Check if we have completed a lap
            // NOTE: we require a certain number of steps since the last lap to avoid incrementing
            // the lap counter because of position noise that can make us jumping rope with the
            // finish line.
            const auto s_thresh = this->get_parameter("s_thresh").as_double();
            if (steps_since_last_lap >= MIN_STEPS_SINCE_LAST_LAP && last_s_proj < s_thresh &&
                s_proj >= s_thresh) {
                completed_laps++;
                steps_since_last_lap = 0;
            } else {
                steps_since_last_lap++;
            }

            // Stop after a certain number of laps
            if (completed_laps >= this->get_parameter("laps").as_int()) {
                this->fsm.header.stamp = this->now();
                this->fsm.state = FSM::NOMINAL_STOPPING;
                this->fsm_pub->publish(this->fsm);
            }
        }
    }

    // Start the system on manual trigger
    Service<std_srvs::srv::Empty>::SharedPtr start_srv;
    void start_cb() {
        if (this->fsm.state == FSM::IDLE) {
            this->fsm.header.stamp = this->now();
            this->fsm.state = FSM::RACING;
            this->fsm_pub->publish(this->fsm);
        }
    }

    // Publish the FSM state periodically
    rclcpp::TimerBase::SharedPtr timer;
    void periodic_fsm_state_publish() {
        this->fsm.header.stamp = this->now();
        this->fsm_pub->publish(this->fsm);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinationNode>();
    try {
        rclcpp::spin(node);
    } catch (std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
