cmake_minimum_required(VERSION 3.8)
project(robot_sdk_driver LANGUAGES CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ------------------------------------------------------------------
# RobotSDK location. Defaults to the sibling directory under the same
# workspace root as this package.
# ------------------------------------------------------------------
if(NOT DEFINED ROBOT_SDK_ROOT)
  get_filename_component(ROBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../genisom_robot_sdk" ABSOLUTE)
endif()
if(NOT EXISTS "${ROBOT_SDK_ROOT}/lib")
  message(FATAL_ERROR "RobotSDK not found at ${ROBOT_SDK_ROOT}. "
                      "Set -DROBOT_SDK_ROOT=<path> to the genisom_robot_sdk directory.")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCH_DIR "aarch64")
else()
  set(ARCH_DIR "x86_64")
endif()

set(ROBOT_SDK_LIB "${ROBOT_SDK_ROOT}/lib/${ARCH_DIR}/librobot_sdk.so")
if(NOT EXISTS "${ROBOT_SDK_LIB}")
  message(FATAL_ERROR "RobotSDK library not found: ${ROBOT_SDK_LIB}")
endif()

add_library(robot_sdk SHARED IMPORTED)
set_target_properties(robot_sdk PROPERTIES
  IMPORTED_LOCATION "${ROBOT_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${ROBOT_SDK_ROOT}/include")

message(STATUS "RobotSDK root: ${ROBOT_SDK_ROOT}")
message(STATUS "RobotSDK lib : ${ROBOT_SDK_LIB}")

# ------------------------------------------------------------------
# Node executable
# ------------------------------------------------------------------
add_executable(robot_sdk_driver
  src/main.cpp
  src/robot_sdk_node.cpp
  src/sdk_client_wrapper.cpp)

target_include_directories(robot_sdk_driver PRIVATE include)
ament_target_dependencies(robot_sdk_driver
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  std_srvs
  tf2_ros
  builtin_interfaces)
target_link_libraries(robot_sdk_driver robot_sdk)

set_target_properties(robot_sdk_driver PROPERTIES
  BUILD_RPATH "${ROBOT_SDK_ROOT}/lib/${ARCH_DIR}"
  INSTALL_RPATH "${ROBOT_SDK_ROOT}/lib/${ARCH_DIR}")

install(TARGETS robot_sdk_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
