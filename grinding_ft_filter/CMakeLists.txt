cmake_minimum_required(VERSION 3.8)
project(grinding_ft_filter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install Python modules
ament_python_install_package(grinding_ft_filter_pkg)

# Install scripts
install(
  PROGRAMS
  grinding_ft_filter_pkg/ft_filter.py
  scripts/noisy_wrench_pub.py  
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()