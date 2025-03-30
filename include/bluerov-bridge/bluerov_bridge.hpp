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

#include <visualization_msgs/msg/marker.hpp>
/**
 * @brief A ROS 2 node that interfaces with ArduPilot/BlueROV using MAVLink protocol.
 *
 * This bridge enables:
 * 1. Manual control by passing velocity commands to ArduSub
 * 2. Waypoint navigation in GUIDED mode (single waypoint or path)
 * 3. Position holding at the last waypoint
 * 
 * Features:
 * - Coordinate transformations between NED (ArduSub) and ENU (ROS)
 * - Automatic mode switching based on flight state
 * - Queue-based waypoint following
 * - Position hold after waypoint completion
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_reached_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kcl_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr data_timer_;         // Timer for MAVLink data reception
    rclcpp::TimerBase::SharedPtr control_loop_timer_; // Timer for manual control
    rclcpp::TimerBase::SharedPtr waypoint_timer_;     // Timer for waypoint navigation

    //--------------------------------------------------------------------------
    // MAVLink Socket / Connection
    //--------------------------------------------------------------------------
    int sock_fd_{-1};                   // UDP socket file descriptor
    struct sockaddr_in remote_addr_{};  // Remote address for sending MAVLink

    // Our GCS system ID
    uint8_t system_id_{255};            // MAVLink system ID (255 = ground station)
    uint8_t component_id_{190};         // MAVLink component ID

    // Autopilot IDs (discovered from heartbeat)
    uint8_t target_system_{0};          // Target system ID (from heartbeat)
    uint8_t target_component_{0};       // Target component ID (from heartbeat)
    bool got_heartbeat_{false};         // Flag indicating if heartbeat was received

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
    Eigen::Matrix<double, 6, 1> velocityDesired_{Eigen::Matrix<double, 6, 1>::Zero()};   // [m/s, m/s, m/s, rad/s, rad/s, rad/s]
    Eigen::Matrix<double, 6, 1> velocityDesiredPwm_{Eigen::Matrix<double, 6, 1>::Zero()}; // [PWM, PWM, PWM, PWM, PWM, PWM]

    /// Velocity limits
    Eigen::Matrix<double, 6, 1> maxVelocities_{2.0, 1.8, 0.55, 2.0, 2.2, 1.85}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]
    Eigen::Matrix<double, 6, 1> minVelocities_{-2.0, -1.8, -0.55, -2.0, -2.2, -1.85}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]

    /// KCL state
    std::string kcl_state_{};

    /// Depth filtering
    bool depth_initialized_{false};
    double depth_filtered_;
    double alpha_depth_;
    
    //--------------------------------------------------------------------------
    // Waypoint Navigation Variables
    //--------------------------------------------------------------------------
    
    /// Queue of waypoints (stored in ENU coordinates)
    std::queue<geometry_msgs::msg::PoseStamped> waypoint_queue_;
    
    /// Current waypoint being navigated to (ENU coordinates)
    geometry_msgs::msg::PoseStamped current_waypoint_;
    
    /// Waypoint navigation state
    bool waypoint_navigation_active_{false}; // Actively following waypoints
    bool waypoint_reached_{false};           // Current waypoint reached
    bool guided_mode_active_{false};         // Vehicle is in GUIDED mode
    
    /// Mode change tracking
    bool mode_change_requested_{false};      // Mode change has been requested
    rclcpp::Time mode_change_request_time_{}; // Time of last mode change request
    uint32_t mode_change_attempts_{0};       // Number of mode change attempts
    
    /// Waypoint acceptance radius (meters)
    double waypoint_acceptance_radius_{0.1};
    
    /// Position hold at last waypoint when queue is empty
    bool position_hold_active_{false};              // Vehicle is in POSHOLD mode
    geometry_msgs::msg::PoseStamped position_hold_waypoint_; // Position being held

    //--------------------------------------------------------------------------
    // Internal Methods
    //--------------------------------------------------------------------------
    /**
     * @brief Initialize MAVLink UDP socket connection
     */
    void initMavlinkConnection();
    
    /**
     * @brief Request data streams from ArduSub
     */
    void requestDataStreams();
    
    /**
     * @brief Receive and process MAVLink data
     */
    void receiveData();
    
    /**
     * @brief Main control loop for manual control mode
     */
    void controlLoop();

    /**
     * @brief Process velocity commands from ROS
     */
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief Process state change requests
     */
    void kclStateCallback(const std_msgs::msg::String::SharedPtr msg);
    
    /**
     * @brief Process single waypoint requests
     */
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief Process path (multiple waypoints) requests
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    
    /**
     * @brief Timer callback for waypoint navigation progress
     */
    void waypointNavigationTimer();
    
    /**
     * @brief Process the next waypoint in the queue
     */
    void processNextWaypoint();
    
    /**
     * @brief Check if current waypoint has been reached
     */
    bool isWaypointReached();
    
    /**
     * @brief Engage position hold mode
     */
    void positionHold();
    
    /**
     * @brief Set ArduSub to GUIDED mode for waypoint navigation
     */
    void setGuidedMode();
    
    /**
     * @brief Send waypoint to ArduSub in NED coordinates
     */
    void sendWaypointToArdupilot(const geometry_msgs::msg::PoseStamped& waypoint);
    
    /**
     * @brief Convert from ENU to NED coordinate system
     */
    void convertENUtoNED(const geometry_msgs::msg::PoseStamped& enu, mavlink_set_position_target_local_ned_t& ned);

    /**
     * @brief Set ArduSub to POSHOLD mode for position maintenance
     */
    void setPosHoldMode();
    
    /**
     * @brief Arm the vehicle in specified mode
     */
    void arm(const std::string& mode = "MANUAL");
    
    /**
     * @brief Disarm the vehicle
     */
    void disarm();

    /**
     * @brief Convert desired velocities to PWM values
     */
    Eigen::VectorXd velocityToPwm(const Eigen::VectorXd& velocities, const Eigen::VectorXd& maxVelocities, const Eigen::VectorXd& minVelocities);
    
    /**
     * @brief Set RC channel PWM values
     */
    void setRcChannelPwm(const Eigen::VectorXd& velocityDesiredPwm);
    
    /**
     * @brief Send a MAVLink message to ArduSub
     */
    void sendMavlinkMessage(const mavlink_message_t& msg);
    
    /**
     * @brief Set the update interval for MAVLink messages
     */
    void setMessageInterval(uint16_t message_id, float frequency_hz);
    
    double calculateYaw(const geometry_msgs::msg::PoseStamped& waypoint);
};

