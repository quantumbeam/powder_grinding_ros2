cmake_minimum_required(VERSION 3.8)
project(grinding_descriptions)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#-- Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  moveit_visual_tools
)

#--- for C++
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp geometry_msgs moveit_ros_planning_interface tf2_geometry_msgs )
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

include_directories(include)
custom_executable(load_planing_scene)

target_link_libraries(load_planing_scene
  moveit_visual_tools::moveit_visual_tools
  ${catkin_LIBRARIES}
)
#--- end C++

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
