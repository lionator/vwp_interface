cmake_minimum_required(VERSION 3.5)
project(vwp_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_auto_add_library(vehicle_status_publisher SHARED 
#  src/vehicle_status_publisher/vehicle_status_publisher.cpp
#)

ament_auto_add_executable(vwp_interface_node
  src/vwp_interface/vwp_interface.cpp
  src/vwp_interface/vwp_interface_node.cpp
)

ament_auto_add_executable(vehicle_status_publisher_node
  src/vehicle_status_publisher/vehicle_status_publisher.cpp
  src/vehicle_status_publisher/vehicle_status_publisher_node.cpp
)

#rclcpp_components_register_node(vehicle_status_publisher
#  PLUGIN "vwp_interface::VehicleStatusPublisher"
#  EXECUTABLE vehicle_status_publisher_node)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
