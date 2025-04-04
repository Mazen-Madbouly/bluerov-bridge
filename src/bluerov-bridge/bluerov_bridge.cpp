#include "bluerov-bridge/bluerov_bridge.hpp"

// C / C++ Includes
#include <chrono>
#include <cmath>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

/**
 * @file bluerov_bridge.cpp
 * @brief Implementation of the BlueROVBridge class that connects ROS2 to ArduSub via MAVLink
 * 
 * This bridge enables:
 * 1. Manual control by passing velocity commands to ArduSub
 * 2. Waypoint navigation in GUIDED mode (single waypoint or path)
 * 3. Position holding at the last waypoint
 * 
 * Communication is via MAVLink over UDP, using the standard ArduSub protocol.
 * Coordinate transformations between NED (ArduSub) and ENU (ROS) are handled.
 */

//=============================================================================
// Constructor
//=============================================================================
BlueROVBridge::BlueROVBridge(const rclcpp::NodeOptions& options)
  : Node("mavlink_bridge", options)
{
  RCLCPP_INFO(this->get_logger(), 
      "Starting BlueROVBridge node (UDP port 14551)...");

  // 1) Initialize the MAVLink UDP connection on local port=14551
  initMavlinkConnection();

  // 2) Setup ROS pubs/subs
  pose_actual_pub_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(
      "/auv/pose_actual", 
      10
  );

  velocity_actual_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/auv/velocity_actual", 
      10
  );

  // Publisher for waypoint reached notification
  waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/auv/waypoint_reached",
      10
  );

  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/auv/velocity_desired",
      10,
      std::bind(&BlueROVBridge::velocityCallback, this, std::placeholders::_1)
  );

  kcl_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/auv/kcl_state",
      10,
      std::bind(&BlueROVBridge::kclStateCallback, this, std::placeholders::_1)
  );

  // Subscribe to waypoint and path topics
  waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/auv/waypoint",
      10,
      std::bind(&BlueROVBridge::waypointCallback, this, std::placeholders::_1)
  );

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/auv/path",
      10,
      std::bind(&BlueROVBridge::pathCallback, this, std::placeholders::_1)
  );

  // 3) Timers
  data_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(125), // ~8Hz
      std::bind(&BlueROVBridge::receiveData, this)
  );

  control_loop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(125), // ~8Hz
      std::bind(&BlueROVBridge::controlLoop, this)
  );

  // Waypoint navigation timer (check progress at 8Hz)
  waypoint_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(125), // 8Hz
      std::bind(&BlueROVBridge::waypointNavigationTimer, this)
  );

  // 4) Initialize parameters
  waypoint_acceptance_radius_ = this->declare_parameter<double>("waypoint_acceptance_radius", 0.1);
  
  RCLCPP_INFO(this->get_logger(), "Waypoint navigation configured with acceptance radius: %.2f meters", 
              waypoint_acceptance_radius_);
}

//=============================================================================
// Destructor
//=============================================================================
BlueROVBridge::~BlueROVBridge()
{
  if (sock_fd_ != -1) {
    close(sock_fd_);
  }
  RCLCPP_INFO(this->get_logger(), "BlueROVBridge node shutting down.");
}

//=============================================================================
// initMavlinkConnection
//   Bind local port=14551. We do not set remote_addr_ until we see autopilot heartbeat.
//=============================================================================
/**
 * @brief Initialize MAVLink UDP connection
 * 
 * This method:
 * 1. Creates a UDP socket bound to port 14551 (standard ArduSub port)
 * 2. Sets up initial remote_addr_ to the simulation address (127.0.0.1)
 * 3. Will update remote_addr_ when the first heartbeat is received
 * 
 * @throws std::runtime_error if socket creation or binding fails
 */
void BlueROVBridge::initMavlinkConnection()
{
  // Create UDP socket
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    const std::string err_msg = "Failed to create UDP socket: " + std::string(strerror(errno));
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }

  // Set up socket options - allow port reuse
  int reuse = 1;
  if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
  }

  // Configure local address to bind to all interfaces (0.0.0.0) on port 14551
  sockaddr_in local_addr;
  std::memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family      = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY; // 0.0.0.0 (listen on all interfaces)
  local_addr.sin_port        = htons(14551);  // Match BlueOS MAVLink endpoint

  // Bind the socket
  if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
    const std::string err_msg = "Socket bind failed on port 14551: " + std::string(strerror(errno));
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    close(sock_fd_);
    throw std::runtime_error(err_msg);
  }

  RCLCPP_INFO(this->get_logger(),
      "Bound to 0.0.0.0:14551, waiting for ArduSub telemetry (will initially send to 127.0.0.1:14551)");

  // Initialize remote address (will be updated when first heartbeat is received)
  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");  // Default for simulation
  remote_addr_.sin_port = htons(14551); // Send commands to ArduPilot on 14551

  // Initialize ArduPilot-related fields
  target_system_   = 0;
  target_component_= 0;
  got_heartbeat_   = false;
}


//=============================================================================
// receiveData()
//   Non-blocking read; parse MAVLink. Once we see autopilot's heartbeat, store
//   that IP/port in remote_addr_, so we can send commands back on same port.
//=============================================================================
void BlueROVBridge::receiveData()
{
  while (true)  // Keep reading until no more data is available
  {
    uint8_t buffer[2048];
    sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    // Non-blocking recv
    ssize_t recsize = recvfrom(
        sock_fd_,
        buffer,
        sizeof(buffer),
        MSG_DONTWAIT,
        reinterpret_cast<struct sockaddr*>(&sender_addr),
        &addr_len
    );

    // If there's no more data (or an error), break out of the while loop
    if (recsize <= 0) {
      break;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    // Parse all bytes in the received packet
    for (ssize_t i = 0; i < recsize; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
        switch (msg.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
          // Ignore if it's our own GCS heartbeat
          if (msg.sysid == system_id_ && msg.compid == component_id_) {
            RCLCPP_INFO(this->get_logger(),
               "Ignoring GCS heartbeat (sys=%d, comp=%d).", msg.sysid, msg.compid);
            break;
          }

          // Extract the heartbeat info
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          // If we haven't yet received the autopilot heartbeat, handle it:
          if (!got_heartbeat_) {
            target_system_    = msg.sysid;
            target_component_ = msg.compid;
            got_heartbeat_    = true;

            // Overwrite remote_addr_ with the sender's IP:port
            remote_addr_ = sender_addr;

            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(sender_addr.sin_addr), ip_str, sizeof(ip_str));
            uint16_t sender_port = ntohs(sender_addr.sin_port);

            RCLCPP_INFO(this->get_logger(),
                "Got AUTOPILOT heartbeat from sys=%d, comp=%d at %s:%d => storing remote_addr_",
                target_system_, target_component_, ip_str, sender_port);

            // Request data streams only once
            requestDataStreams();
          }
          
          // Check the current vehicle mode
          // For ArduSub, customMode contains the mode number
          uint32_t current_mode = hb.custom_mode;
          
          // Log the current mode value when it changes (helps with debugging)
          static uint32_t last_mode = UINT32_MAX;
          if (current_mode != last_mode) {
            RCLCPP_INFO(this->get_logger(), "Vehicle mode changed: %u", current_mode);
            last_mode = current_mode;
          }
          
          // ArduSub mode values (may vary by firmware version)
          // Typical values are:
          // STABILIZE = 0, MANUAL = 19, ALT_HOLD = 2, GUIDED = 4, POSHOLD = 16
          
          // Track mode changes
          // GUIDED mode is 4 in ArduSub
          bool is_in_guided_mode = (current_mode == 4);
          
          // POSHOLD mode is 16 in ArduSub
          bool is_in_poshold_mode = (current_mode == 16);
          
          // Check if we have a pending mode change request
          if (mode_change_requested_) {
            // Check if the current mode matches what we requested
            bool mode_change_succeeded = false;
            
            if (requested_mode_ == "GUIDED" && is_in_guided_mode) {
              mode_change_succeeded = true;
            } else if (requested_mode_ == "POSHOLD" && is_in_poshold_mode) {
              mode_change_succeeded = true;
            } else if (requested_mode_ == "MANUAL" && current_mode == 19) {
              mode_change_succeeded = true;
            } else if (requested_mode_ == "STABILIZE" && current_mode == 0) {
              mode_change_succeeded = true;
            } else if (requested_mode_ == "ALT_HOLD" && current_mode == 2) {
              mode_change_succeeded = true;
            }
            
            // If mode change succeeded or we've been waiting too long, clear the request
            rclcpp::Duration timeout(3, 0); // 3 seconds timeout
            bool timed_out = (this->now() - mode_change_request_time_) > timeout;
            
            if (mode_change_succeeded) {
              RCLCPP_INFO(this->get_logger(), "Mode change to %s confirmed!", requested_mode_.c_str());
              mode_change_requested_ = false;
              mode_change_attempts_ = 0;
              
              // Update the state flags based on the confirmed mode
              if (requested_mode_ == "GUIDED") {
                guided_mode_active_ = true;
                position_hold_active_ = false;
              } else if (requested_mode_ == "POSHOLD") {
                position_hold_active_ = true;
                guided_mode_active_ = false;
              } else {
                guided_mode_active_ = false;
                position_hold_active_ = false;
              }
            } else if (timed_out) {
              RCLCPP_WARN(this->get_logger(), 
                  "Mode change to %s timed out after %d attempts!", 
                  requested_mode_.c_str(), mode_change_attempts_);
              mode_change_requested_ = false;
              
              // Don't reset mode_change_attempts_ here to limit future attempts
            }
          }
          
          // If we think we're in GUIDED but the vehicle isn't, or vice versa
          if (guided_mode_active_ != is_in_guided_mode) {
            if (is_in_guided_mode) {
              RCLCPP_INFO(this->get_logger(), "Vehicle is now in GUIDED mode!");
              guided_mode_active_ = true;
              position_hold_active_ = false;
              mode_change_attempts_ = 0;
            } else if (guided_mode_active_ && !is_in_guided_mode && !mode_change_requested_) {
              // Only log this if we're actively trying to change modes and there's no pending request
              RCLCPP_WARN(this->get_logger(), 
                  "Vehicle is not in GUIDED mode (current mode: %u)", current_mode);
            }
          }
          
          // Check if in POSHOLD mode
          if (position_hold_active_ != is_in_poshold_mode) {
            if (is_in_poshold_mode) {
              RCLCPP_INFO(this->get_logger(), "Vehicle is now in POSHOLD mode!");
              position_hold_active_ = true;
              guided_mode_active_ = false;
              mode_change_attempts_ = 0;
            } else if (position_hold_active_ && !is_in_poshold_mode && !mode_change_requested_) {
              // Only log this if there's no pending mode change request
              RCLCPP_WARN(this->get_logger(), 
                  "Vehicle is not in POSHOLD mode (current mode: %u)", current_mode);
            }
          }
          
          break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);

          // Store orientation (roll, pitch, yaw)
          poseActual_[3] = attitude.roll;
          poseActual_[4] = attitude.pitch;
          poseActual_[5] = attitude.yaw;

          // Store angular velocity (rollspeed, pitchspeed, yawspeed)
          velocityActual_[3] = attitude.rollspeed;
          velocityActual_[4] = attitude.pitchspeed;
          velocityActual_[5] = attitude.yawspeed;

          if (!home_pose_set_) {
            poseHome_[3] = poseActual_[3];
            poseHome_[4] = poseActual_[4];
            poseHome_[5] = poseActual_[5];
          }
          break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
          mavlink_local_position_ned_t pos_ned;
          mavlink_msg_local_position_ned_decode(&msg, &pos_ned);

          // Position in NED
          poseActual_[0] = pos_ned.x;
          poseActual_[1] = pos_ned.y;
          poseActual_[2] = pos_ned.z;

          // Velocity in NED
          velocityActual_[0] = pos_ned.vx;
          velocityActual_[1] = pos_ned.vy;
          velocityActual_[2] = pos_ned.vz;

          if (!home_pose_set_) {
            poseHome_[0] = poseActual_[0];
            poseHome_[1] = poseActual_[1];
            poseHome_[2] = poseActual_[2];
            home_pose_set_ = true;
          }
          break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&msg, &ack);
          
          // Log ACK messages regardless of the command type
          if (ack.command == MAV_CMD_DO_SET_MODE) {
            if (ack.result == MAV_RESULT_ACCEPTED) {
              RCLCPP_INFO(this->get_logger(), "Mode change accepted by ArduSub");
            } else {
              // Log the specific error code for better diagnostics
              const char* error_str = "Unknown";
              switch (ack.result) {
                case MAV_RESULT_DENIED: error_str = "DENIED"; break;
                case MAV_RESULT_UNSUPPORTED: error_str = "UNSUPPORTED"; break;
                case MAV_RESULT_FAILED: error_str = "FAILED"; break;
                case MAV_RESULT_TEMPORARILY_REJECTED: error_str = "TEMPORARILY_REJECTED"; break;
                default: error_str = "UNKNOWN"; break;
              }
              
              RCLCPP_WARN(this->get_logger(), 
                  "Mode change REJECTED by ArduSub (result=%s).", error_str);
            }
          } else {
            // Only log non-mode change ACKs at DEBUG level to reduce noise
            RCLCPP_DEBUG(this->get_logger(),
                "CMD_ACK received: command=%u, result=%u",
                ack.command, ack.result);
          }
          break;
        }

        default:
          // Ignore other message types
          break;
        } // end switch
      }   // end if (mavlink_parse_char)
    }     // end for (recsize)
  }       // end while(true)
}


/****************************************************************************
 * requestDataStreams
 *   Called once after we get the first heartbeat. Instead of calling
 *   request_data_stream_send(), we use MAV_CMD_SET_MESSAGE_INTERVAL
 *   for each message ID we care about (e.g. ATTITUDE, LOCAL_POSITION_NED).
 ***************************************************************************/
void BlueROVBridge::requestDataStreams()
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(),
        "requestDataStreams() called but no autopilot heartbeat yet!");
    return;
  }

  // Example: we want 8 Hz for attitude (#30) and local position (#32).
  setMessageInterval(MAVLINK_MSG_ID_ATTITUDE,           8.0f); // #30
  setMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 8.0f); // #32

  RCLCPP_INFO(this->get_logger(),
      "Configured message intervals for ATTITUDE(#30) and LOCAL_POSITION_NED(#32) at 8 Hz.");
}



//=============================================================================
// controlLoop()
//=============================================================================
void BlueROVBridge::controlLoop()
{
  if (guided_mode_active_) {
    return;   // Skip control loop if guided mode is active
  }

  // 1) Check for valid home position
  if (std::isnan(poseHome_[0]) || std::isnan(poseHome_[1]) || std::isnan(poseHome_[2])) {
    RCLCPP_WARN(this->get_logger(), "Home position not set yet. Skipping pose calculation.");
    return;
  }
  if (!got_heartbeat_ /* or !home_pose_set_, etc. */) {
    RCLCPP_WARN_ONCE(this->get_logger(), 
        "No heartbeat/position data yet; skipping control loop...");
    return;  
  }

  // Print PWM values
  //RCLCPP_INFO(this->get_logger(), "PWM Values: [%f, %f, %f, %f, %f, %f]",
  //  velocityDesiredPwm_[0], velocityDesiredPwm_[1], velocityDesiredPwm_[2],
  //  velocityDesiredPwm_[3], velocityDesiredPwm_[4], velocityDesiredPwm_[5]);

  //--------------------------------------------------------------------------
  // POSE: Transform from NED -> ENU
  //--------------------------------------------------------------------------

  // Calculate relative position in NED
  double relative_x_ned = poseActual_[0] - poseHome_[0];
  double relative_y_ned = poseActual_[1] - poseHome_[1];
  double relative_z_ned = poseActual_[2] - poseHome_[2];

  // NED -> ENU rotation
  Eigen::Matrix3d R_NED_to_ENU;
  R_NED_to_ENU << 0,  1,  0,
                  1,  0,  0,
                  0,  0, -1;

  // Transform position from NED to ENU
  Eigen::Vector3d position_ned(relative_x_ned, relative_y_ned, relative_z_ned);
  Eigen::Vector3d position_enu = R_NED_to_ENU * position_ned;


  double transformed_x = position_enu(0);
  double transformed_y = position_enu(1);
  double transformed_z = position_enu(2);

  double roll  = poseActual_[3];
  double pitch = poseActual_[4];
  double yaw   = poseActual_[5];

  // Publish PoseStamped (same logic as Python version)
  auv_core_helper::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "world";  // matches Python's 'world'

  // Note how the Python code swapped X/Y in the final output
  pose_msg.x = transformed_x;
  pose_msg.y = transformed_y;
  pose_msg.z = transformed_z;

  // roll, pitch, yaw 
  pose_msg.roll  = roll;
  pose_msg.pitch = pitch;
  pose_msg.yaw   = yaw;

  pose_actual_pub_->publish(pose_msg);

  //--------------------------------------------------------------------------
  // VELOCITY: Transform from NED -> ENU and publish
  //--------------------------------------------------------------------------

  // Linear velocity in NED
  Eigen::Vector3d vel_ned(
      velocityActual_[0],
      velocityActual_[1],
      velocityActual_[2]
  );
  // Transform to ENU
  Eigen::Vector3d vel_enu = R_NED_to_ENU * vel_ned;

  // Fill the Twist message
  geometry_msgs::msg::Twist velocity_msg;
  // Again, note the swapping to match the X/Y logic above
  velocity_msg.linear.x = vel_enu(0);
  velocity_msg.linear.y = vel_enu(1);
  velocity_msg.linear.z = vel_enu(2);

  // Angular velocities: we typically keep rollspeed, pitchspeed, yawspeed as-is
  velocity_msg.angular.x = velocityActual_[3];
  velocity_msg.angular.y = velocityActual_[4];
  velocity_msg.angular.z = velocityActual_[5];

  // Publish it
  velocity_actual_pub_->publish(velocity_msg);

  //--------------------------------------------------------------------------
  // CONTROL: Send RC overrides based on velocityDesired_
  //--------------------------------------------------------------------------

  // int forward_pwm  = velocityToPwm(velocityDesired_[0]); // forward/back
  // int lateral_pwm  = velocityToPwm(velocityDesired_[1]); // left/right
  // int vertical_pwm = velocityToPwm(velocityDesired_[2]); // up/down
  // int roll_pwm     = velocityToPwm(velocityDesired_[3]); // roll rate
  // int pitch_pwm    = velocityToPwm(velocityDesired_[4]); // pitch rate
  // int yaw_pwm      = velocityToPwm(velocityDesired_[5]); // yaw rate

  velocityDesiredPwm_ = velocityToPwm(velocityDesired_, maxVelocities_, minVelocities_);
  setRcChannelPwm(velocityDesiredPwm_);

  // Then set the RC channels in the same order:
  // 1=pitch, 2=roll, 3=throttle, 4=yaw, 5=forward, 6=lateral
  // setRcChannelPwm(1, pitch_pwm);
  // setRcChannelPwm(2, roll_pwm);
  // setRcChannelPwm(3, vertical_pwm);
  // setRcChannelPwm(4, yaw_pwm);
  // setRcChannelPwm(5, forward_pwm);
  // setRcChannelPwm(6, lateral_pwm);
}


//=============================================================================
// velocityCallback
//=============================================================================
void BlueROVBridge::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Store the desired velocity
  velocityDesired_[0] = msg->linear.x;
  velocityDesired_[1] = msg->linear.y;
  velocityDesired_[2] = msg->linear.z;
  velocityDesired_[3] = msg->angular.x;
  velocityDesired_[4] = msg->angular.y;
  velocityDesired_[5] = msg->angular.z;
}

//=============================================================================
// kclStateCallback
//   - If state="IDLE": disarm
//   - If state="ARM": arm without changing mode
//   - If state="MANUAL", "STABILIZE", etc.: set flight mode (possibly while staying armed)
//   - If "PATH_FOLLOWING": reset home pose
//   - If "WAYPOINT_NAVIGATION": set GUIDED mode for waypoint navigation
//=============================================================================
void BlueROVBridge::kclStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == kcl_state_) {
    return;
  }

  // Handle arming/disarming separately from mode changes
  if (msg->data == "IDLE") {
    disarm();
    waypoint_navigation_active_ = false;
    guided_mode_active_ = false;
    position_hold_active_ = false;
    
    // Clear waypoint queue
    std::queue<geometry_msgs::msg::PoseStamped> empty;
    std::swap(waypoint_queue_, empty);
  } else if (msg->data == "ARM") {
    // Arm without changing mode
    arm("CURRENT");
  } else if (msg->data == "MANUAL" || msg->data == "STABILIZE" || 
           msg->data == "ALT_HOLD" || msg->data == "ACRO") {
    // Set flight mode without changing arm state
    setFlightMode(msg->data);
  }

  // Handle special states for navigation
  if (msg->data == "PATH_FOLLOWING") {
    poseHome_ = poseActual_;
    home_pose_set_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Home pose set: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
      poseHome_[0], poseHome_[1], poseHome_[2],
      poseHome_[3], poseHome_[4], poseHome_[5]
    );

    // Switch to GUIDED mode for path following
    setGuidedMode();
    
    RCLCPP_INFO(this->get_logger(), 
        "PATH_FOLLOWING activated: Using GUIDED mode for waypoint navigation");
  } else if (msg->data == "WAYPOINT_NAVIGATION") {
    // Set home pose if not already set
    poseHome_.setZero();
    home_pose_set_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Home pose set for waypoint navigation: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
      poseHome_[0], poseHome_[1], poseHome_[2],
      poseHome_[3], poseHome_[4], poseHome_[5]
    );
    
    // Switch to GUIDED mode for waypoint navigation (without changing arm state)
    setGuidedMode();
    
    // Process any waypoints in the queue
    if (!waypoint_queue_.empty() && !waypoint_navigation_active_) {
      processNextWaypoint();
    }
  } else if (msg->data == "POSITION_HOLD") {
    // Set home pose if not already set
    if (!home_pose_set_) {
      poseHome_ = poseActual_;
      home_pose_set_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Home pose set for position hold: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
        poseHome_[0], poseHome_[1], poseHome_[2],
        poseHome_[3], poseHome_[4], poseHome_[5]
      );
    }
    
    // Use current position as hold position
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.stamp = this->now();
    current_pose.header.frame_id = "world";
    
    // Store the current position in ENU for position hold
    // We won't need to use these directly as POSHOLD mode will handle position maintenance
    current_pose.pose.position.x = 0.0;  // Use relative position at current location
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.0;
    
    // Set as the position to hold
    position_hold_waypoint_ = current_pose;
    
    // Switch to POSHOLD mode (without changing arm state)
    setPosHoldMode();
    
    RCLCPP_INFO(this->get_logger(), "Manually entered POSITION_HOLD mode at current position");
  }

  kcl_state_ = msg->data;
}

//=============================================================================
// arm
//=============================================================================
void BlueROVBridge::arm(const std::string& mode)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot arm yet; no autopilot heartbeat discovered!");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
      "Arming vehicle (sys=%d, comp=%d)...",
      target_system_, target_component_);

  // MAV_CMD_COMPONENT_ARM_DISARM
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      1.0, // param1=1 => arm
      0,0,0,0,0,0
  );
  sendMavlinkMessage(msg);
  RCLCPP_INFO(this->get_logger(), "Arm command sent.");
  
  // If a mode is specified, set it
  if (!mode.empty() && mode != "CURRENT") {
    setFlightMode(mode);
  }
}

//=============================================================================
// setFlightMode
// Sets the flight mode without changing arm state
//=============================================================================
void BlueROVBridge::setFlightMode(const std::string& mode)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot set mode yet; no autopilot heartbeat discovered!");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
      "Setting flight mode=%s (sys=%d, comp=%d)...",
      mode.c_str(), target_system_, target_component_);

  // Map mode string to ArduSub custom mode number
  int32_t custom_mode = 19; // Default to MANUAL=19
  if (mode == "MANUAL") {
    custom_mode = 19;
  } else if (mode == "STABILIZE") {
    custom_mode = 0;
  } else if (mode == "ALT_HOLD") {
    custom_mode = 2;
  } else if (mode == "GUIDED") {
    custom_mode = 4;  // GUIDED mode
  } else if (mode == "POSHOLD") {
    custom_mode = 16; // POSHOLD mode
  } else {
    RCLCPP_WARN(this->get_logger(), 
        "Unknown mode '%s', defaulting to MANUAL", mode.c_str());
  }
  
  // Send the mode change command
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_DO_SET_MODE,
      0, // confirmation
      1,  // param1: Mode, as defined by MAV_MODE enum (1 = MODE_GUIDED)
      static_cast<float>(custom_mode),  // param2: Custom mode - ArduSub mode
      0,  // param3: Custom sub-mode - not used for ArduSub
      0, 0, 0, 0  // param4-7 unused
  );
  
  // Send multiple times for reliability (UDP is unreliable)
  constexpr int NUM_RETRIES = 3;
  constexpr int RETRY_DELAY_MS = 20;
  
  for (int i = 0; i < NUM_RETRIES; i++) {
    sendMavlinkMessage(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
  }
  
  RCLCPP_INFO(this->get_logger(), 
      "Set mode command sent (%s, sent %d times).", mode.c_str(), NUM_RETRIES);
      
  // Set the requested mode and track the request time
  // Note: we don't immediately set guided_mode_active_/position_hold_active_ flags
  // Instead, we wait for confirmation via heartbeat
  requested_mode_ = mode;
  mode_change_requested_ = true;
  mode_change_request_time_ = this->now();
  mode_change_attempts_++;
  
  // Log the mode change attempt number
  if (mode_change_attempts_ > 1) {
    RCLCPP_INFO(this->get_logger(), "Mode change attempt #%d", mode_change_attempts_);
  }
}

//=============================================================================
// disarm
//=============================================================================
void BlueROVBridge::disarm()
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot disarm yet; no autopilot heartbeat discovered!");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
      "Disarming vehicle (sys=%d, comp=%d)...",
      target_system_, target_component_);

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      0.0, // param1=0 => disarm
      0,0,0,0,0,0
  );
  sendMavlinkMessage(msg);
}

//=============================================================================
// velocityToPwm
//=============================================================================
/**
 * @brief Convert desired vehicle velocities to PWM values for ArduSub
 * 
 * This function maps the desired velocity values to the appropriate PWM range
 * for ArduSub's RC override system. It applies:
 * 1. Range clamping to ensure values don't exceed vehicle capabilities
 * 2. Non-linear scaling for better sensitivity at low speeds
 * 3. Mapping to the PWM range expected by ArduSub (1000-2000)
 *
 * @param velocities The desired velocities (x,y,z,roll,pitch,yaw)
 * @param maxVelocities Maximum allowable velocities for each axis
 * @param minVelocities Minimum allowable velocities for each axis
 * @return Eigen::VectorXd PWM values for each channel
 * @throws std::invalid_argument if input vectors have incorrect dimensions
 */
Eigen::VectorXd BlueROVBridge::velocityToPwm(const Eigen::VectorXd& velocities, 
                                             const Eigen::VectorXd& maxVelocities, 
                                             const Eigen::VectorXd& minVelocities)
{
    // Validate input size
    constexpr int EXPECTED_SIZE = 6;
    if (velocities.size() != EXPECTED_SIZE || 
        maxVelocities.size() != EXPECTED_SIZE || 
        minVelocities.size() != EXPECTED_SIZE) {
        throw std::invalid_argument("velocities, maxVelocities, and minVelocities must be 6x1 vectors");
    }

    Eigen::VectorXd pwmValues(EXPECTED_SIZE);

    // PWM constants
    constexpr double PWM_CENTER = 1500.0;
    constexpr double PWM_RANGE = 500.0;
    constexpr double PWM_MIN = 1000.0;
    constexpr double PWM_MAX = 2000.0;
    constexpr double NONLINEAR_FACTOR = 0.8; // Power factor for non-linear scaling (less than 1.0 gives more sensitivity)

    for (int i = 0; i < EXPECTED_SIZE; ++i) {
        // Clamp velocity to the specified range for the given index
        double clampedVelocity = std::clamp(velocities(i), minVelocities(i), maxVelocities(i));

        // Map the clamped velocity to the range [-1, 1]
        double normalizedVelocity = 0.0;
        double range = maxVelocities(i) - minVelocities(i);
        if (std::abs(range) > 1e-6) { // Avoid division by zero
            normalizedVelocity = (clampedVelocity - minVelocities(i)) / range * 2.0 - 1.0;
        }

        // Apply non-linear scaling to increase sensitivity for small velocities
        if (normalizedVelocity > 0) {
            normalizedVelocity = std::pow(normalizedVelocity, NONLINEAR_FACTOR);
        } else if (normalizedVelocity < 0) {
            normalizedVelocity = -std::pow(std::abs(normalizedVelocity), NONLINEAR_FACTOR);
        }

        // Map the normalized velocity to the PWM range
        pwmValues(i) = PWM_CENTER + normalizedVelocity * PWM_RANGE;
        
        // Ensure the PWM output is within the valid range
        pwmValues(i) = std::clamp(pwmValues(i), PWM_MIN, PWM_MAX);
    }

    return pwmValues;
}


//=============================================================================
// setRcChannelPwm
//=============================================================================
void BlueROVBridge::setRcChannelPwm(const Eigen::VectorXd& velocityDesiredPwm)
{
    // Check the size of the input vector
    if (velocityDesiredPwm.size() != 6) {
        RCLCPP_ERROR(this->get_logger(),
                     "setRcChannelPwm: velocityDesiredPwm must have exactly 6 elements.");
        return;
    }

    // Initialize all channels to "do not change" (65535).
    uint16_t rc_channel_values[18];
    for (int i = 0; i < 18; ++i) {
        rc_channel_values[i] = 65535; // Do not override these channels
    }

    // Map velocityDesiredPwm to the correct channel order:
    //
    // Channel 1 -> pitch    = velocityDesiredPwm(4)
    // Channel 2 -> roll     = velocityDesiredPwm(3)
    // Channel 3 -> vertical = velocityDesiredPwm(2)
    // Channel 4 -> yaw      = velocityDesiredPwm(5)
    // Channel 5 -> forward  = velocityDesiredPwm(0)
    // Channel 6 -> lateral  = velocityDesiredPwm(1)
    rc_channel_values[0] = static_cast<uint16_t>(velocityDesiredPwm(4));  // pitch
    rc_channel_values[1] = static_cast<uint16_t>(velocityDesiredPwm(3));  // roll
    rc_channel_values[2] = static_cast<uint16_t>(velocityDesiredPwm(2));  // vertical
    rc_channel_values[3] = static_cast<uint16_t>(velocityDesiredPwm(5));  // yaw
    rc_channel_values[4] = static_cast<uint16_t>(velocityDesiredPwm(0));  // forward
    rc_channel_values[5] = static_cast<uint16_t>(velocityDesiredPwm(1));  // lateral

    // Send MAVLink RC override message
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(
        system_id_,
        component_id_,
        &msg,
        target_system_,
        target_component_,
        rc_channel_values[0],
        rc_channel_values[1],
        rc_channel_values[2],
        rc_channel_values[3],
        rc_channel_values[4],
        rc_channel_values[5],
        rc_channel_values[6],
        rc_channel_values[7],
        rc_channel_values[8],
        rc_channel_values[9],
        rc_channel_values[10],
        rc_channel_values[11],
        rc_channel_values[12],
        rc_channel_values[13],
        rc_channel_values[14],
        rc_channel_values[15],
        rc_channel_values[16],
        rc_channel_values[17]
    );

    sendMavlinkMessage(msg);
}



//=============================================================================
// sendMavlinkMessage
//=============================================================================
void BlueROVBridge::sendMavlinkMessage(const mavlink_message_t& msg)
{
  if (sock_fd_ < 0) {
    return;
  }

  uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t msg_len = mavlink_msg_to_send_buffer(tx_buffer, &msg);

  ssize_t bytes_sent = sendto(
      sock_fd_,
      tx_buffer,
      msg_len,
      0,
      reinterpret_cast<const struct sockaddr*>(&remote_addr_),
      sizeof(remote_addr_)
  );
  if (bytes_sent < 0) {
    RCLCPP_ERROR(this->get_logger(),
        "Failed sending MAVLink message (errno=%d).", errno);
  }
}

//=============================================================================
// setMessageInterval
//=============================================================================
void BlueROVBridge::setMessageInterval(uint16_t message_id, float frequency_hz)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(),
        "setMessageInterval(%d) called, but no autopilot heartbeat yet!", message_id);
    return;
  }

  // Interval in microseconds
  float interval_us = 1.0e6f / frequency_hz;  

  mavlink_message_t msg;
  // param1 = message_id
  // param2 = desired interval in microseconds
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_SET_MESSAGE_INTERVAL,
      0, // confirmation
      static_cast<float>(message_id), // param1
      interval_us,                    // param2
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f    // param3..7 unused
  );

  sendMavlinkMessage(msg);
  RCLCPP_INFO(this->get_logger(),
      "Requested message #%d at %.1f Hz (%.0f us).",
      message_id, frequency_hz, interval_us);
}

//=============================================================================
// waypointCallback
// Handles a single waypoint received from the /auv/waypoint topic
//=============================================================================
void BlueROVBridge::waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), 
      "Received new waypoint: x=%.2f, y=%.2f, z=%.2f", 
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Clear any existing waypoints
  std::queue<geometry_msgs::msg::PoseStamped> empty;
  std::swap(waypoint_queue_, empty);
  
  // Add the new waypoint to the queue
  waypoint_queue_.push(*msg);
  
  // If already in position hold mode, exit it to start waypoint navigation
  if (position_hold_active_) {
    position_hold_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Exiting position hold mode to start waypoint navigation");
  }
  
  // If not currently navigating to a waypoint, start the process
  if (!waypoint_navigation_active_) {
    processNextWaypoint();
  } else {
    RCLCPP_INFO(this->get_logger(), "Waypoint added to queue. Will navigate after current waypoint is reached.");
  }
}

//=============================================================================
// pathCallback
// Handles a sequence of waypoints received as a path from the /auv/path topic
//=============================================================================
void BlueROVBridge::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path. No waypoints to follow.");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", msg->poses.size());
  
  // Clear existing waypoints
  std::queue<geometry_msgs::msg::PoseStamped> empty;
  std::swap(waypoint_queue_, empty);
  
  // Add all waypoints from the path to the queue
  for (const auto& pose : msg->poses) {
    waypoint_queue_.push(pose);
  }
  
  // If already in position hold mode, exit it to start waypoint navigation
  if (position_hold_active_) {
    position_hold_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Exiting position hold mode to start path following");
  }
  
  // If not currently navigating to a waypoint, start the process
  if (!waypoint_navigation_active_) {
    processNextWaypoint();
  } else {
    RCLCPP_INFO(this->get_logger(), 
        "Path with %zu waypoints added to queue. Will navigate after current waypoint is reached.", 
        msg->poses.size());
  }
}

//=============================================================================
// waypointNavigationTimer
// Periodically checks if current waypoint is reached and manages waypoint queue
//=============================================================================
void BlueROVBridge::waypointNavigationTimer()
{
  if (!got_heartbeat_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waypoint navigation active but no autopilot connection yet");
    return;
  }
  
  // Stop here if we're in position hold mode
  if (position_hold_active_) {
    return;  // No waypoint navigation while in position hold
  }

  if (!waypoint_navigation_active_) {
    return;  // No active navigation, nothing to do
  }
  
  // Only proceed if we're actually in GUIDED mode
  if (!guided_mode_active_) {
    // Check if we have already tried setting GUIDED mode too many times
    const int MAX_MODE_CHANGE_ATTEMPTS = 5;
    
    if (mode_change_attempts_ >= MAX_MODE_CHANGE_ATTEMPTS) {
      RCLCPP_ERROR(this->get_logger(), 
          "Failed to set GUIDED mode after %d attempts. Canceling waypoint navigation.",
          mode_change_attempts_);
      waypoint_navigation_active_ = false;
      return;
    }
    
    // Check if we're already waiting for a mode change to complete
    if (mode_change_requested_ && requested_mode_ == "GUIDED") {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Waiting for GUIDED mode change to complete... (attempt #%d)",
          mode_change_attempts_);
      return;
    }
    
    RCLCPP_WARN(this->get_logger(), 
        "Waypoint navigation active but not in GUIDED mode. Attempting to set GUIDED mode...");
    setGuidedMode();
    return;
  }

  // **** Re-send the current waypoint setpoint continuously ****
  sendWaypointToArdupilot(current_waypoint_);
  
  // Check if the current waypoint has been reached
  if (isWaypointReached()) {
    RCLCPP_INFO(this->get_logger(), "Waypoint reached!");
    
    // Publish waypoint reached notification
    std_msgs::msg::Bool reached_msg;
    reached_msg.data = true;
    waypoint_reached_pub_->publish(reached_msg);
    
    // Reset the flag
    waypoint_reached_ = false;
    
    // Process the next waypoint if available
    if (!waypoint_queue_.empty()) {
      processNextWaypoint();
    } else {
      RCLCPP_INFO(this->get_logger(), "Waypoint queue empty. Switching to POSHOLD mode. ");
      waypoint_navigation_active_ = false;
      setPosHoldMode();  // Switch to POSHOLD mode directly
    }
  }
}

//=============================================================================
// isWaypointReached
// Determines if vehicle has reached the current waypoint based on position
//=============================================================================
bool BlueROVBridge::isWaypointReached()
{
  if (!home_pose_set_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Home pose not set yet. Cannot determine if waypoint is reached.");
    return false;
  }

  // Calculate relative position in NED
  double relative_x_ned = poseActual_[0] - poseHome_[0];
  double relative_y_ned = poseActual_[1] - poseHome_[1];
  double relative_z_ned = poseActual_[2] - poseHome_[2];

  // Transform from NED to ENU for comparison with waypoint
  // This is the inverse of the conversion in convertENUtoNED
  // ENU.x = NED.y
  // ENU.y = NED.x
  // ENU.z = -NED.z
  double current_x_enu = relative_y_ned;
  double current_y_enu = relative_x_ned;
  double current_z_enu = -relative_z_ned;
  
  // Calculate distance to waypoint in 3D space
  double dx = current_x_enu - current_waypoint_.pose.position.x;
  double dy = current_y_enu - current_waypoint_.pose.position.y;
  double dz = current_z_enu - current_waypoint_.pose.position.z;
  
  double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
  
  // Print current and waypoint positions
  RCLCPP_INFO(this->get_logger(), 
    "Current ENU: (%.2f, %.2f, %.2f) | Waypoint ENU: (%.2f, %.2f, %.2f)",
    current_x_enu, current_y_enu, current_z_enu,
    current_waypoint_.pose.position.x, 
    current_waypoint_.pose.position.y,
    current_waypoint_.pose.position.z);

  RCLCPP_DEBUG(this->get_logger(), 
      "Distance to waypoint: %.2f meters (acceptance radius: %.2f)",
      distance, waypoint_acceptance_radius_);
  
  // Waypoint is reached if within acceptance radius
  return (distance <= waypoint_acceptance_radius_);
}

//=============================================================================
// processNextWaypoint
// Takes the next waypoint from the queue and sends it to ArduPilot
//=============================================================================
void BlueROVBridge::processNextWaypoint()
{
  if (waypoint_queue_.empty()) {
    RCLCPP_WARN(this->get_logger(), "processNextWaypoint() called with empty queue");
    return;
  }

  // Ensure we're in GUIDED mode (but don't repeatedly try if already trying)
  if (!guided_mode_active_ && (!mode_change_requested_ || requested_mode_ != "GUIDED")) {
    setGuidedMode();
    
    // We'll wait for the mode change to complete in the next timer cycle
    return;
  }
  
  // Send the waypoint to ArduPilot
  current_waypoint_ = waypoint_queue_.front();  // Get the next waypoint from the queue
  waypoint_queue_.pop();                        // Remove the waypoint from the queue
  sendWaypointToArdupilot(current_waypoint_);   // Send the waypoint to ArduPilot
  
  // Update state
  waypoint_navigation_active_ = true;
  waypoint_reached_ = false;
  position_hold_active_ = false;
}

//=============================================================================
// setGuidedMode
// Switches ArduPilot to GUIDED mode for waypoint navigation
//=============================================================================
void BlueROVBridge::setGuidedMode()
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot set GUIDED mode yet; no autopilot heartbeat discovered!");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
      "Setting GUIDED mode (sys=%d, comp=%d)...",
      target_system_, target_component_);

  setFlightMode("GUIDED");
}

//=============================================================================
// setPosHoldMode
// Switches ArduPilot to POSHOLD mode to hold position
//=============================================================================
/**
 * @brief Switch ArduPilot to POSHOLD mode to hold position
 * 
 * This method:
 * 1. Sends the SET_MODE command to ArduSub with custom_mode=16 (POSHOLD)
 * 2. Updates the internal state to reflect position hold is active
 * 3. Disables waypoint navigation and guided mode
 * 4. Stores the current position for reference
 * 
 * Position hold mode is used after completing waypoints to maintain position.
 */
void BlueROVBridge::setPosHoldMode() {
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot set POSHOLD mode; no autopilot heartbeat!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engaging POSHOLD mode at current position.");
  setFlightMode("POSHOLD");
  
  // Save current position as the position hold waypoint
  if (home_pose_set_) {
    position_hold_waypoint_ = current_waypoint_;
    RCLCPP_INFO(this->get_logger(), 
        "Storing position hold at: (%.2f, %.2f, %.2f)",
        position_hold_waypoint_.pose.position.x,
        position_hold_waypoint_.pose.position.y,
        position_hold_waypoint_.pose.position.z);
  }
}

//=============================================================================
// sendWaypointToArdupilot
// Converts a waypoint to MAVLink SET_POSITION_TARGET_LOCAL_NED message
//=============================================================================
/**
 * @brief Send a waypoint to ArduPilot in GUIDED mode
 * 
 * This function converts a ROS waypoint (in ENU coordinates) to a MAVLink
 * SET_POSITION_TARGET_LOCAL_NED message (in NED coordinates) and sends it
 * to ArduPilot.
 * 
 * @param waypoint The waypoint position in ENU coordinates
 */
void BlueROVBridge::sendWaypointToArdupilot(const geometry_msgs::msg::PoseStamped& waypoint)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send waypoint yet; no autopilot heartbeat discovered!");
    return;
  }
  
  // Convert waypoint from ENU to NED coordinate system for ArduPilot
  mavlink_set_position_target_local_ned_t position_target;
  convertENUtoNED(waypoint, position_target);
  
  // Create the MAVLink message
  mavlink_message_t msg;
  mavlink_msg_set_position_target_local_ned_encode(
      system_id_,
      component_id_,
      &msg,
      &position_target
  );
  
  // Send the message
  sendMavlinkMessage(msg);
  
  RCLCPP_INFO(this->get_logger(), 
      "Sent waypoint to ArduPilot: NED(%.2f, %.2f, %.2f)",
      position_target.x, position_target.y, position_target.z);
}

//=============================================================================
// convertENUtoNED
// Converts waypoint from ENU coordinates to NED coordinates for ArduPilot
//=============================================================================
/**
 * @brief Convert waypoint from ENU to NED coordinate system
 * 
 * Coordinate conversion from ROS (ENU) to ArduPilot (NED):
 * NED.x = ENU.y  (North = East)
 * NED.y = ENU.x  (East = North) 
 * NED.z = -ENU.z (Down = -Up)
 * 
 * @param enu Input waypoint in ENU coordinates
 * @param ned Output structure in NED coordinates for MAVLink
 */
void BlueROVBridge::convertENUtoNED(const geometry_msgs::msg::PoseStamped& enu, 
                                   mavlink_set_position_target_local_ned_t& ned)
{
  // Clear the structure
  memset(&ned, 0, sizeof(ned));
  
  // Set header information
  ned.time_boot_ms = static_cast<uint32_t>(this->now().nanoseconds() / 1000000);
  ned.target_system = target_system_;
  ned.target_component = target_component_;
  ned.coordinate_frame = MAV_FRAME_LOCAL_NED;
  
  // Create mask for position only (ignore velocity, acceleration, yaw and yaw rate)
  // Each bit set to 1 means "ignore this dimension"
  //ned.type_mask = 0b111111111000; // Use position only
  
  ned.type_mask= 0b0000001111000000;
  // Convert ENU to NED coordinates
  ned.x = enu.pose.position.y;  // North = East (ENU.y -> NED.x)
  ned.y = enu.pose.position.x;  // East = North (ENU.x -> NED.y)
  ned.z = -enu.pose.position.z; // Down = -Up (negative ENU.z -> NED.z)
  
  // Set velocities and accelerations to zero (not used with the defined type_mask)
  ned.vx = 0.0f;
  ned.vy = 0.0f;
  ned.vz = 0.0f;
  ned.afx = 0.0f;
  ned.afy = 0.0f;
  ned.afz = 0.0f;
  ned.yaw = 0.0f;
  ned.yaw_rate = 0.0f;
}
