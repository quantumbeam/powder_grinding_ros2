cmake_minimum_required(VERSION 3.8)
project(grinding_ft_filter)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Interface generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WrenchStatistics.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# --- Install Python package ---
ament_python_install_package(grinding_ft_filter_pkg)

# --- Install executables ---
install(
  PROGRAMS
    grinding_ft_filter_pkg/ft_filter.py
    grinding_ft_filter_pkg/wrench_statistics.py
    scripts/noisy_wrench_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files ---
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
