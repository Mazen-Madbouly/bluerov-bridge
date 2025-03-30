#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <string>

class UserInput : public rclcpp::Node {
public:
    UserInput() : Node("user_input") {
        // Publisher to set flight mode
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("auv/kcl_state", 10);

        // Publisher for waypoints
        waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/auv/waypoint", 10);

        RCLCPP_INFO(this->get_logger(), "User Input Node started.");
        showMenu();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;

    void showMenu() {
        while (rclcpp::ok()) {
            std::cout << "\n=== BlueROV Menu ===\n";
            std::cout << "1. Set Flight Mode\n";
            std::cout << "2. Set Waypoint\n";
            std::cout << "Select an option: ";

            int choice;
            std::cin >> choice;
            std::cin.ignore();

            switch (choice) {
                case 1:
                    setFlightMode();
                    break;
                case 2:
                    setWaypoint();
                    break;
                default:
                    std::cout << "Invalid option. Try again.\n";
                    break;
            }
        }
    }

    void setFlightMode() {
        std::cout << "\nAvailable Modes:\n";
        std::cout << "1. GUIDED\n";
        std::cout << "2. MANUAL\n";
        std::cout << "3. IDLE\n";
        //std::cout << "4. POSHOLD\n";
        std::cout << "Select mode: ";

        int mode_choice;
        std::cin >> mode_choice;
        std::cin.ignore();

        std::string mode;
        switch (mode_choice) {
            case 1: mode = "WAYPOINT_NAVIGATION"; break;
            case 2: mode = "MANUAL"; break;
            case 3: mode = "IDLE"; break;
            //case 4: mode = "POSHOLD"; break;
            default:
                std::cout << "Invalid choice. Returning to menu.\n";
                return;
        }

        std_msgs::msg::String msg;
        msg.data = mode;
        mode_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Flight mode set to: %s", mode.c_str());
    }

    void setWaypoint() {
        double x, y, z;
        std::cout << "\nEnter Waypoint Coordinates (x y z): ";
        std::cin >> x >> y >> z;
        std::cin.ignore();

        geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header.stamp = this->now();
        waypoint.header.frame_id = "world";
        waypoint.pose.position.x = x;
        waypoint.pose.position.y = y;
        waypoint.pose.position.z = z;

        waypoint_pub_->publish(waypoint);
        RCLCPP_INFO(this->get_logger(), "Waypoint set to: (%.2f, %.2f, %.2f)", x, y, z);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UserInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
