cmake_minimum_required(VERSION 3.8)
project(spot_msgs VERSION 0.1.0 LANGUAGES C CXX)
#######################################################################
# Build flags
#######################################################################
add_compile_options(-Wall -Wextra -Wno-unused-function -O3 -march=native)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ROS 2 ament framework
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(NOT TARGET QHULL::QHULL)
  find_library(QHULL_LIB qhull_r REQUIRED)
  add_library(QHULL::QHULL SHARED IMPORTED)
  set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_LOCATION ${QHULL_LIB})
endif()


# Generate custom messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootState.msg"
  "msg/FootStateArray.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Third-party libraries
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Executable node
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  message_filters
  sensor_msgs
  geometry_msgs
)
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node
  ${EIGEN3_LIBRARIES}
)

# Install executable
install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
