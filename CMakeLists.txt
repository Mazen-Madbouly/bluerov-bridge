cmake_minimum_required(VERSION 3.8)
project(bluerov-bridge)

################################################################################
# Set compiler flags
################################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-pedantic -Wno-address-of-packed-member)



################################################################################
# Find packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auv_core_helper REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(mavlink REQUIRED)
find_package(geographic_msgs REQUIRED)

pkg_check_modules(LIBCONFIGPP REQUIRED libconfig++)

# Prefer shared libraries over static ones
set(CMAKE_FIND_LIBRARY_SUFFIXES .so .a)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${auv_core_helper_INCLUDE_DIRS}
  ${LIBCONFIGPP_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)


################################################################################
# Link directories (if needed)
################################################################################
link_directories(${CMAKE_INSTALL_PREFIX}/auv_core_helper/lib)

################################################################################
# RPATH settings
################################################################################
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################################################################
# Source / Executable definitions
################################################################################
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

add_executable(bluerov_bridge_node
    src/bluerov_bridge_node.cpp
    src/bluerov-bridge/bluerov_bridge.cpp
)

add_executable(user_input src/user_input.cpp)

################################################################################
# Dependencies and linking
################################################################################
ament_target_dependencies(bluerov_bridge_node
  rclcpp
  visualization_msgs
  tf2_ros
  geometry_msgs
  Eigen3
  sensor_msgs
  tf2_geometry_msgs
  auv_core_helper
  rclcpp_lifecycle
  nav_msgs
  mavlink
  geographic_msgs
)

ament_target_dependencies(user_input rclcpp std_msgs geographic_msgs)

target_link_libraries(bluerov_bridge_node
  ${auv_core_helper_LIBRARIES}
  ${LIBCONFIGPP_LIBRARIES}
  ${mavlink_LIBRARIES}
)

################################################################################
# Installation
################################################################################
install(TARGETS bluerov_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS user_input DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
  OPTIONAL
)

install(DIRECTORY
  launch
  config
  models
  param
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

################################################################################
# Testing
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Finalize
################################################################################
ament_package()