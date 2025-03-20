#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS 2 message headers
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"         // For waypoint poses
#include "nav_msgs/msg/path.hpp"                     // For path of waypoints
#include "std_msgs/msg/bool.hpp"                     // For waypoint reached notification

// We will use Eigen for the NED->ENU transform
#include <Eigen/Dense>

// Include the MAVLink C headers
extern "C" {
    #include <mavlink/v2.0/common/mavlink.h>
}

// Standard libraries for sockets (UDP example)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <queue>                                     // For waypoint queue

/**
 * @brief A ROS 2 node that interfaces with ArduPilot/BlueROV using MAVLink protocol.
 *  Supports velocity control, waypoint navigation, and guided mode operations.
 */
class BlueROVBridge : public rclcpp::Node
{
public:
    /// Constructor
    BlueROVBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /// Destructor
    ~BlueROVBridge();

private:
    //--------------------------------------------------------------------------
    // ROS Publishers & Subscribers
    //--------------------------------------------------------------------------
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr pose_actual_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_actual_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_reached_pub_;  // NEW: Publish when waypoint is reached

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kcl_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;  // NEW: Single waypoint subscriber
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;  // NEW: Path subscriber for multiple waypoints

    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr data_timer_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;  // NEW: Timer to check waypoint progress

    //--------------------------------------------------------------------------
    // MAVLink Socket / Connection
    //--------------------------------------------------------------------------
    int sock_fd_{-1};                  
    struct sockaddr_in remote_addr_{};

    // Our GCS system ID
    uint8_t system_id_{255};
    uint8_t component_id_{190};

    // Autopilot IDs (discovered from heartbeat)
    uint8_t target_system_{0};
    uint8_t target_component_{0};

    //--------------------------------------------------------------------------
    // State Variables
    //--------------------------------------------------------------------------

    /// [x, y, z, roll, pitch, yaw] in NED
    Eigen::Matrix<double, 6, 1> poseActual_{Eigen::Matrix<double, 6, 1>::Zero()};

    /// [vx, vy, vz, rollspeed, pitchspeed, yawspeed]
    Eigen::Matrix<double, 6, 1> velocityActual_{Eigen::Matrix<double, 6, 1>::Zero()};

    /// Home pose in NED
    Eigen::Matrix<double, 6, 1> poseHome_{Eigen::Matrix<double, 6, 1>::Zero()};
    bool home_pose_set_{false};

    /// Desired velocity [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
    Eigen::Matrix<double, 6, 1> velocityDesired_{Eigen::Matrix<double, 6, 1>::Zero()}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]
    Eigen::Matrix<double, 6, 1> velocityDesiredPwm_{Eigen::Matrix<double, 6, 1>::Zero()}; // [PWM, PWM, PWM, PWM, PWM, PWM]

    //max and min velocity limits
    Eigen::Matrix<double, 6, 1> maxVelocities_{2.0, 1.8, 0.55, 2.0, 2.2, 1.85}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]
    Eigen::Matrix<double, 6, 1> minVelocities_{-2.0, -1.8, -0.55, -2.0, -2.2, -1.85}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]

    /// KCL state
    std::string kcl_state_{};

    bool got_heartbeat_{false};

    bool depth_initialized_{false};
    double depth_filtered_;
    double alpha_depth_;
    
    //--------------------------------------------------------------------------
    // NEW: Waypoint Navigation Variables
    //--------------------------------------------------------------------------
    
    /// Queue of waypoints (stored in ENU coordinates)
    std::queue<geometry_msgs::msg::PoseStamped> waypoint_queue_;
    
    /// Current waypoint being navigated to (ENU coordinates)
    geometry_msgs::msg::PoseStamped current_waypoint_;
    
    /// Waypoint navigation state
    bool waypoint_navigation_active_{false};
    bool waypoint_reached_{false};
    bool guided_mode_active_{false};
    
    /// Waypoint acceptance radius (meters)
    double waypoint_acceptance_radius_{0.5};
    
    /// Position hold at last waypoint when queue is empty
    bool position_hold_active_{false};
    geometry_msgs::msg::PoseStamped position_hold_waypoint_;

    //--------------------------------------------------------------------------
    // Internal Methods
    //--------------------------------------------------------------------------
    void initMavlinkConnection();
    void requestDataStreams();
    void receiveData();
    void controlLoop();

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void kclStateCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // NEW: Waypoint handling methods
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void waypointNavigationTimer();
    void processNextWaypoint();
    bool isWaypointReached();
    void positionHold();
    
    // NEW: MAVLink guided mode methods
    void setGuidedMode();
    void sendWaypointToArdupilot(const geometry_msgs::msg::PoseStamped& waypoint);
    void convertENUtoNED(const geometry_msgs::msg::PoseStamped& enu, mavlink_set_position_target_local_ned_t& ned);

    void setPosHoldMode();  // NEW: POSHOLD mode method

    void arm(const std::string& mode = "MANUAL");
    void disarm();

    Eigen::VectorXd velocityToPwm(const Eigen::VectorXd& velocities, const Eigen::VectorXd& maxVelocities, const Eigen::VectorXd& minVelocities);
    void setRcChannelPwm(const Eigen::VectorXd& velocityDesiredPwm);
    void sendMavlinkMessage(const mavlink_message_t& msg);
    void setMessageInterval(uint16_t message_id, float frequency_hz);
};

