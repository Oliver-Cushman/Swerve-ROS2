cmake_minimum_required(VERSION 3.20)
project(swerve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swerve_interfaces REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(input_node src/input_node.cpp)
add_executable(central_node src/central_node.cpp)

target_link_libraries(input_node PUBLIC ${swerve_interfaces_TARGETS} ${SDL2_LIBRARIES} rclcpp::rclcpp)
target_link_libraries(central_node PUBLIC ${swerve_interfaces_TARGETS} ${SDL2_LIBRARIES} rclcpp::rclcpp)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    input_node
    central_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
