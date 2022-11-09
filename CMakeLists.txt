cmake_minimum_required(VERSION 3.5)
project(tof_ros2cpp)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib;/opt/glog;/opt/protobuf;/opt/websockets")
list(APPEND AMENT_PREFIX_PATH "/usr/local;/opt/glog;/opt/protobuf;/opt/websockets")
set(CMAKE_INSTALL_RPATH "/usr/local/lib;/opt/websockets/lib")
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_package(aditof REQUIRED)
find_package(Libwebsockets REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)

find_library(TOFI_COMPUTE
  NAMES tofi_compute
  PATHS /usr/lib)

set(SOURCES ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_executable(tof_ros2cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_camera_node.cpp ${SOURCES})


target_include_directories(tof_ros2cpp PUBLIC ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)
target_sources(tof_ros2cpp INTERFACE ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)

ament_target_dependencies(tof_ros2cpp image_transport rclcpp std_msgs aditof glog)

target_include_directories(tof_ros2cpp PUBLIC "${ADITOF_INCLUDE_DIRS}")
target_include_directories(tof_ros2cpp PUBLIC "${LIBWEBSOCKETS_INCLUDE_DIRS}")
target_link_libraries(tof_ros2cpp "${ADITOF_LIBRARIES}")
target_link_libraries(tof_ros2cpp aditof::aditof)
target_link_libraries(tof_ros2cpp websockets_shared)


install(TARGETS
tof_ros2cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# Copying config files and depth compute libraries from sdk/config, sdk/lib folders to build/examples/first-frame and build/first-frame/Debug or Release folders    
add_custom_command( TARGET tof_ros2cpp POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_CURRENT_SOURCE_DIR}/../../config
    COMMENT "Copying cfgs and libs to build config "
)