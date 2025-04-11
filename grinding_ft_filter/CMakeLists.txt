cmake_minimum_required(VERSION 3.8)
project(grinding_ft_filter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(
  PROGRAMS
  grinding_ft_filter/ft_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()