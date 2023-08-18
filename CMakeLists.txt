cmake_minimum_required(VERSION 3.8)
project(lbr_end_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fri_vendor REQUIRED)
find_package(FRIClient REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(end_control_node src/end_control_node.cpp)
ament_target_dependencies(end_control_node
  Eigen3
  kdl_parser
  lbr_fri_ros2
  lbr_fri_msgs
  orocos_kdl
  rclcpp
  tf2_ros
  tf2
)

target_link_libraries(end_control_node
  FRIClient::FRIClient
)

target_include_directories(end_control_node
  PRIVATE src
)

add_executable(test_end_publisher src/test_end_publisher.cpp)
ament_target_dependencies(test_end_publisher
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
)

install(
  TARGETS test_end_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS end_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
