#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include <iostream>
#include <string>

class UserInput : public rclcpp::Node {
public:
    UserInput() : Node("user_input") {
        // Publisher to set flight mode
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("auv/kcl_state", 10);

        // Publisher for waypoints
        waypoint_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/auv/waypoint", 10);

        RCLCPP_INFO(this->get_logger(), "User Input Node started.");
        showMenu();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr waypoint_pub_;

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
        double lat, lon, alt;
        std::cout << "\nEnter Waypoint Coordinates (Lat Lon Alt): ";
        std::cin >> lat >> lon >> alt;
        std::cin.ignore();
        // TODO: input check

        geographic_msgs::msg::GeoPoseStamped waypoint;
        waypoint.header.stamp = this->now();
        waypoint.header.frame_id = "world";

       
        waypoint.pose.position.latitude = lat;
        waypoint.pose.position.longitude = lon;
        waypoint.pose.position.altitude = alt;

        // Pubblica il waypoint
        waypoint_pub_->publish(waypoint);
        RCLCPP_INFO(this->get_logger(), "Waypoint set: Lat=%.7f, Lon=%.7f, Alt=%.2f", lat, lon, alt);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UserInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
