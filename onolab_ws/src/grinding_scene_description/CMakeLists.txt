cmake_minimum_required(VERSION 3.8)
project(grinding_scene_description)

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


# Find package from package.xml
ament_auto_find_build_dependencies()

# For shared library
# The shared library load_planning_scene is created from the # source file load_planning_scene.cpp
ament_auto_add_library(load_planning_scene src/load_planning_scene.cpp)
# The shared library load_planning_scene in 
rclcpp_components_register_nodes(load_planning_scene "load_planning_scene::LoadPlanningScene")

# Add executable
# Execute add_executable, target_include_directories, target_link_libraries, and ament_target_dependencies together
ament_auto_add_executable(test_load_planning_scene src/test_load_planning_scene.cpp)
ament_auto_add_executable(load_planning_scene src/main.cpp)
# ament_auto_add_executable(new_load_planning_scene src/new_load_planning_scene.cpp)
# ament_auto_add_executable(load_planning_scene_py scripts/load_planning_scene.py)

# Add message and service
ament_auto_generate_code()

# For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install files
install(DIRECTORY
  launch
  config
  # scripts/load_planning_scene.py
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
