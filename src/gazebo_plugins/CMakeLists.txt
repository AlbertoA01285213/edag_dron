cmake_minimum_required(VERSION 3.8)
project(gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${ignition-gazebo6_INCLUDE_DIRS}
  ${ignition-plugin1_INCLUDE_DIRS}
  ${ignition-transport11_INCLUDE_DIRS}
)

add_library(SubmarineDynamics SHARED
  src/SubmarineDynamics.cpp
  src/uuv_dynamic_model.cpp
)

target_link_libraries(SubmarineDynamics
  ${ignition-gazebo6_LIBRARIES}
  ignition-plugin1::all
  ignition-transport11::core
  Eigen3::Eigen
)

ament_target_dependencies(SubmarineDynamics
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS SubmarineDynamics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
