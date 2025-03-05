import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from auv_core_helper.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class EKFPoseSubscriber(Node):
    def __init__(self):
        super().__init__('ekf_pose_subscriber')

        # MAVLink Connection
        self.connection = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.get_logger().info("Waiting for a heartbeat...")
        self.connection.wait_heartbeat()
        self.get_logger().info("Heartbeat received!")



        #variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Home position variables (to store the initial position)
        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.home_roll = None
        self.home_pitch = None
        self.home_yaw = None

        # Desired velocities (initialized to 0)
        self.desired_linear_x = 0.0
        self.desired_linear_y = 0.0
        self.desired_linear_z = 0.0
        self.desired_angular_x = 0.0
        self.desired_angular_y = 0.0
        self.desired_angular_z = 0.0

        # Publisher for PoseStamped messages
        self.pose_actual_publisher = self.create_publisher(PoseStamped, '/auv/pose_actual', 10)

        # MAVLink data streams
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1
        )
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,
            1
        )

        # Timers for data and control loops
        self.data_timer = self.create_timer(0.125, self.receive_data)  # 100 Hz
        self.control_loop_timer = self.create_timer(0.125, self.control_loop)  # 10 Hz

        # Subscriber for desired velocity
        self.velocity_subscriber = self.create_subscription(
            Twist, '/auv/velocity_desired', self.velocity_callback, 10)

        # Subscriber for KCL state (to handle arm/disarm)
        self.subscription_state = self.create_subscription(
            String, '/auv/kcl_state', self.kcl_state_callback, 10)

        # Initialize MAVLink master for sending commands
        self.master = self.connection

        # Initialize KCL state
        self.kcl_state = None

    def arm(self, mode='MANUAL'):
        """Arm the vehicle and set its flight mode.

        Args:
            mode (str): The desired flight mode ('Manual', 'Acro', etc.).
        """
        self.get_logger().info(f"Arming the vehicle and setting mode to {mode}...")

        # Send the arm command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm (1 to arm, 0 to disarm)
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        self.get_logger().info("Arm command sent.")

        # Retrieve valid modes dynamically
        mode_mapping = self.master.mode_mapping()
        # {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'CIRCLE': 7, 'SURFACE': 9, 'POSHOLD': 16, 'MANUAL': 19}

        if mode not in mode_mapping:
            self.get_logger().error(f"Unsupported mode: {mode}")
            return

        custom_mode = mode_mapping[mode]

        # Set the flight mode
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode
        )
        self.get_logger().info(f"Set mode command sent to {mode}.")

    def disarm(self):
        """Send MAVLink command to disarm the vehicle."""
        self.get_logger().info("Disarming the vehicle...")

        # Send the disarm command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # Disarm (param1 = 0)
            0, 0, 0, 0, 0, 0
        )

    def receive_data(self):
        """Continuously receive data from MAVLink."""
        msg = self.connection.recv_match(type=['ATTITUDE', 'LOCAL_POSITION_NED'], blocking=False)

        if msg:
            if msg.get_type() == 'ATTITUDE':
                # Update last angles
                self.current_roll =  msg.roll
                self.current_pitch = msg.pitch
                self.current_yaw = msg.yaw

                if self.home_roll is None and self.home_pitch is None and self.home_yaw is None:
                    self.home_roll = self.current_roll
                    self.home_pitch = self.current_pitch
                    self.home_yaw = self.current_yaw

            elif msg.get_type() == 'LOCAL_POSITION_NED':
                self.current_x = msg.x
                self.current_y = msg.y
                self.current_z = msg.z
                # Store home position if not set
                if self.home_x is None and self.home_y is None and self.home_z is None:
                    self.home_x = self.current_x
                    self.home_y = self.current_y
                    self.home_z = self.current_z

    def control_loop(self):
        """Main control loop to calculate and publish relative pose."""

        if self.home_x is None or self.home_y is None or self.home_z is None:
            self.get_logger().warn("Home position not set yet. Skipping pose calculation.")
            return

        # Calculate relative position in NED
        relative_x_ned = self.current_x - self.home_x
        relative_y_ned = self.current_y - self.home_y
        relative_z_ned = self.current_z - self.home_z

        # NED to ENU transformation matrix (3x3)
        R_NED_to_ENU = np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])

        # Transform position from NED to ENU
        position_ned = np.array([relative_x_ned, relative_y_ned, relative_z_ned])
        position_enu = R_NED_to_ENU @ position_ned
        transformed_x, transformed_y, transformed_z = position_enu

        # Transform orientation from NED to ENU
        rotation_ned = R.from_euler('xyz', [self.current_roll, self.current_pitch, self.current_yaw])


        # Define the NED to ENU rotation as a rotation object
        rotation_transform = R.from_matrix(R_NED_to_ENU)

        # Apply the transformation to the orientation
        rotation_enu = rotation_transform * rotation_ned

        # Convert the transformed rotation back to Euler angles
        print('current yaw', self.current_yaw)
        transformed_roll, transformed_pitch, transformed_yaw = rotation_enu.as_euler('xyz')
        print('transformed yaw', transformed_yaw)

        # Create and populate PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'  # Ensure this matches RViz's fixed frame

        # Assign transformed position
        pose_msg.x = transformed_y
        pose_msg.y = transformed_x
        pose_msg.z = transformed_z

        # Assign unwrapped Euler angles
        pose_msg.roll = self.current_roll
        pose_msg.pitch = self.current_pitch
        pose_msg.yaw = self.current_yaw

        self.pose_actual_publisher.publish(pose_msg)


        # -------------------- Compute PWM signals velocities --------------------
        forward_pwm = self.velocity_to_pwm(self.desired_linear_x)    # Forward/backward in body frame X
        lateral_pwm = self.velocity_to_pwm(self.desired_linear_y)    # Left/right in body frame Y
        vertical_pwm = self.velocity_to_pwm(self.desired_linear_z)   # Up/down in body frame Z
        roll_pwm = self.velocity_to_pwm(self.desired_angular_x)      # Roll rate
        pitch_pwm = self.velocity_to_pwm(self.desired_angular_y)     # Pitch rate
        yaw_pwm = self.velocity_to_pwm(self.desired_angular_z)       # Yaw rate

        # Set RC channels
        # 1	Pitch
        # 2	Roll
        # 3	Throttle
        # 4	Yaw
        # 5	Forward
        # 6	Lateral

        self.set_rc_channel_pwm(1, 1900)
        self.set_rc_channel_pwm(2, roll_pwm)
        self.set_rc_channel_pwm(3, vertical_pwm)
        self.set_rc_channel_pwm(4, yaw_pwm)
        self.set_rc_channel_pwm(5, forward_pwm)
        self.set_rc_channel_pwm(6, lateral_pwm)

    def velocity_callback(self, msg):
        """Callback for velocity desired topic."""
        self.desired_linear_x = msg.linear.x
        self.desired_linear_y = msg.linear.y
        self.desired_linear_z = msg.linear.z
        self.desired_angular_x = msg.angular.x
        self.desired_angular_y = msg.angular.y
        self.desired_angular_z = msg.angular.z

    def velocity_to_pwm(self, velocity):
        """
        Map velocity from -1 to 1 to PWM values between 1100 and 1900.
        """
        # Clamp velocity between -1 and 1
        velocity = max(min(velocity, 1.0), -1.0)
        pwm = int(1500 + (velocity * 400))
        # Ensure PWM is within valid range
        pwm = max(min(pwm, 1900), 1100)
        return pwm

    def kcl_state_callback(self, msg):
        """Callback function for /auv/kcl_state topic."""
        # Only arm/disarm if the state changes
        if msg.data != self.kcl_state:
            if msg.data == "IDLE":
                self.disarm()
            else:
                self.arm()

            # Check for state change to PATH_FOLLOWING and reset home pose
            if msg.data == "PATH_FOLLOWING":
                # Reset home pose
                self.home_x = self.current_x
                self.home_y = self.current_y
                self.home_z = self.current_z
                self.home_roll = self.current_roll
                self.home_pitch = self.current_pitch
                self.home_yaw = self.current_yaw
                self.get_logger().info(f"Home pose set: x={self.home_x}, y={self.home_y}, z={self.home_z}, "
                                       f"roll={self.home_roll}, pitch={self.home_pitch}, yaw={self.home_yaw}")

            # Update kcl_state to the new state
            self.kcl_state = msg.data

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            self.get_logger().warn("Channel does not exist.")
            return

        # Create a list of 18 channels initialized to 65535 (do not change)
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm  # Set the PWM for the desired channel

        # Send RC override command
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values
        )

def main(args=None):
    rclpy.init(args=args)
    ekf_pose_subscriber = EKFPoseSubscriber()
    rclpy.spin(ekf_pose_subscriber)
    ekf_pose_subscriber.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
