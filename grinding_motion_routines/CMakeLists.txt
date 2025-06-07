cmake_minimum_required(VERSION 3.8)
project(grinding_motion_routines)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Find package from package.xml
ament_auto_find_build_dependencies()


#--- for C++

# Add executable
# Execute add_executable, target_include_directories, target_link_libraries, and ament_target_dependencies together
ament_auto_add_executable(moveit_example_node src/moveit_example_node.cpp)

# Add message and service
ament_auto_generate_code()


#-- for Python
# This macro is necessary to install the Python scripts with ament_python(and setup.py)
# ament_python_install_package(${PROJECT_NAME})

# This is necessary to install the Python scripts with ament_cmake
install(
  PROGRAMS
  ${PROJECT_NAME}/display_marker.py
  ${PROJECT_NAME}/display_tf.py
  ${PROJECT_NAME}/grinding_motion_generator.py
  ${PROJECT_NAME}/grinding_motion_primitive.py
  scripts/test_display_tf.py
  scripts/test_display_marker.py
  scripts/grinding_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

#-- for Install Directry
# Install launch files
install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()















