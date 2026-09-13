cmake_minimum_required(VERSION 3.8)
project(custom_nav LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robots_dog_msgs REQUIRED)

add_executable(custom_nav_node
  src/custom_nav_node.cpp
  src/path_planner.cpp)

target_include_directories(custom_nav_node PRIVATE include)

ament_target_dependencies(custom_nav_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  robots_dog_msgs)

install(TARGETS custom_nav_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
