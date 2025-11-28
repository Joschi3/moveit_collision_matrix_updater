cmake_minimum_required(VERSION 3.8)
project(moveit_collision_matrix_updater)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(moveit_setup_srdf_plugins REQUIRED)

add_executable(
  moveit_collision_matrix_updater src/moveit_collision_matrix_updater.cpp
                                  src/moveit_collision_matrix_updater_main.cpp)

target_include_directories(
  moveit_collision_matrix_updater
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  moveit_collision_matrix_updater
  rclcpp
  std_msgs
  moveit_core
  srdfdom
  urdf
  moveit_setup_framework
  moveit_setup_srdf_plugins)

install(TARGETS moveit_collision_matrix_updater DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
