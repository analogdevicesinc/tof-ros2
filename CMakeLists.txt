cmake_minimum_required(VERSION 3.4)
project(tof_ros2cpp)
# Set Cmake standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
message("Build ROS2 binding for ADI TOF Camera")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# option(tof_ros2cpp_ENABLE_SDKBUILD "Enable ToF SDK build" ON)
# option(tof_ros2cpp_SUPERBUILD "Build ${PROJECT_NAME} and the projects it depends on." ON)

# if (tof_ros2cpp_ENABLE_SDKBUILD)
#   include("${CMAKE_CURRENT_SOURCE_DIR}/SuperBuild.cmake")
#   return()
# endif()

# set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc")
list(APPEND CMAKE_PREFIX_PATH "/usr/local;/opt/glog;/opt/protobuf;/opt/websockets" ${ADITOF_CMAKE_PREFIX_PATH})

find_package(aditof REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include/${PROJECT_NAME} ${aditof_INCLUDE_DIRS})

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_node.cpp)

add_executable(aditof_camera_node ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_node.cpp ${SOURCES})
target_sources(aditof_camera_node INTERFACE ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})

target_link_libraries(aditof_camera_node aditof::aditof)
# target_link_libraries(aditof_camera_node colcon_LIBRARIES)
ament_target_dependencies(aditof_camera_node rclcpp std_msgs)

install(TARGETS
aditof_camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()