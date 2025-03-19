#include <iostream>       // For std::cerr, std::endl
#include <memory>         // For std::make_shared
#include <string>         // For std::string
#include "bluerov-bridge/bluerov_bridge.hpp" // Include bluerov-bridge class definition
#include "rclcpp/rclcpp.hpp"             // For ROS 2 API

[[nodiscard]] int main(int argc, char** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create node options and initialize the BlueROVBridge node
    rclcpp::NodeOptions options;
    auto node = std::make_shared<BlueROVBridge>(options);

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
