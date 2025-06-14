cmake_minimum_required(VERSION 3.8)
project(grinding_robot_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp_components REQUIRED)


# Find package from package.xml
ament_auto_find_build_dependencies()

#--- for C++

# The shared library load_planning_scene is created from the source file load_planning_scene.cpp
ament_auto_add_library(dynamic_tf_broadcaster_node SHARED src/dynamic_tf_broadcaster_node.cpp)
ament_auto_add_library(static_tf_broadcaster_node SHARED src/static_tf_broadcaster_node.cpp)
# The shared library load_planning_scene in 
rclcpp_components_register_nodes(dynamic_tf_broadcaster_node "grinding_robot_control::DynamicTFBroadcaster")
rclcpp_components_register_nodes(static_tf_broadcaster_node "grinding_robot_control::StaticTFBroadcaster")


# Add executable
# Execute add_executable, target_include_directories, target_link_libraries, and ament_target_dependencies together
# ament_auto_add_executable(dynamic_tf_broadcaster_node src/dynamic_tf_broadcaster_node.cpp)

# Add message and service
# ament_auto_generate_code()

# For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#-- for Python
# This macro is necessary to install the Python scripts with ament_python(and setup.py)
# ament_python_install_package(${PROJECT_NAME})

# This is necessary to install the Python scripts with ament_cmake
install(PROGRAMS
  ${PROJECT_NAME}/JTC_helper.py
  ${PROJECT_NAME}/moveit_helper.py
  scripts/test_trac_ik.py
  scripts/test_pinocchio.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

#-- for Install Directry
install(DIRECTORY
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
