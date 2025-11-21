cmake_minimum_required(VERSION 3.20)
project(swerve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swerve_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SDL2 REQUIRED sdl2)
include_directories(${SDL2_INCLUDE_DIRS})

# Nodes
add_executable(InputNode src/InputNode.cpp)
add_executable(CentralNode src/CentralNode.cpp)
add_executable(KinematicsNode src/KinematicsNode.cpp)

# Link targets
target_link_libraries(InputNode
  PUBLIC ${swerve_interfaces_TARGETS} ${SDL2_LIBRARIES} rclcpp::rclcpp
)

target_link_libraries(CentralNode
  PUBLIC ${swerve_interfaces_TARGETS} rclcpp::rclcpp
)

target_link_libraries(KinematicsNode
  PUBLIC
    ${swerve_interfaces_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  InputNode
  CentralNode
  KinematicsNode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
