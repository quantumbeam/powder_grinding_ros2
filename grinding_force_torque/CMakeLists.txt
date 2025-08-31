cmake_minimum_required(VERSION 3.8)
project(grinding_force_torque)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Find package from package.xml
ament_auto_find_build_dependencies()

# --- WrenchFilter Component ---
add_library(wrench_filter_component SHARED src/wrench_filter_node.cpp)
ament_target_dependencies(
  wrench_filter_component
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
  Eigen3
)
rclcpp_components_register_nodes(wrench_filter_component "grinding_force_torque::WrenchFilter")
install(
  TARGETS wrench_filter_component
  DESTINATION lib
)

# --- WrenchConverter Component ---
add_library(wrench_converter_component SHARED src/wrench_converter_node.cpp)
ament_target_dependencies(
  wrench_converter_component
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
rclcpp_components_register_nodes(wrench_converter_component "grinding_force_torque::WrenchConverter")
install(
  TARGETS wrench_converter_component
  DESTINATION lib
)

# --- Interface generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WrenchStatistics.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# --- Install Python package ---
# This macro is necessary to install the Python scripts with ament_python(and setup.py)
# ament_python_install_package(${PROJECT_NAME})

# This is necessary to install the Python scripts with ament_cmake
install(
  PROGRAMS
    ${PROJECT_NAME}/wrench_statistics.py
    scripts/noisy_wrench_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files ---
install(
  DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
