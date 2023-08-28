cmake_minimum_required(VERSION 3.5)
project(tof_ros2cpp)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib;/opt/glog;/opt/protobuf;/opt/websockets")
list(APPEND AMENT_PREFIX_PATH "/usr/local;/opt/glog;/opt/protobuf;/opt/websockets")
set(CMAKE_INSTALL_RPATH "/usr/local/lib;/opt/glog/lib;/opt/websockets/lib")

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
find_package(sensor_msgs REQUIRED)
find_package(aditof REQUIRED)
find_package(Libwebsockets REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)

find_library(TOFI_COMPUTE
  NAMES tofi_compute
  PATHS /usr/lib)

set(SOURCE ${CMAKE_SOURCE_DIR}/include/tof-ros2)
set(SOURCES ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)

FILE(GLOB_RECURSE HeaderFiles ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp/*.h)
file(GLOB SOURCES SourceFiles ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(tof_ros2cpp_node ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_camera_node.cpp ${SOURCES} ${HeaderFiles} ${SourceFiles})

target_include_directories(tof_ros2cpp_node PUBLIC ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)
target_sources(tof_ros2cpp_node INTERFACE ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)

ament_target_dependencies(tof_ros2cpp_node rclcpp std_msgs glog sensor_msgs)

target_include_directories(tof_ros2cpp_node PUBLIC "${ADITOF_INCLUDE_DIRS}")
target_include_directories(tof_ros2cpp_node PUBLIC "${LIBWEBSOCKETS_INCLUDE_DIRS}")
target_link_libraries(tof_ros2cpp_node "${ADITOF_LIBRARIES}")
target_link_libraries(tof_ros2cpp_node aditof)
target_link_libraries(tof_ros2cpp_node websockets_shared)
target_link_libraries(tof_ros2cpp_node tofi_compute)
target_link_libraries(tof_ros2cpp_node tofi_config)
target_link_libraries(tof_ros2cpp_node glog::glog)

install(TARGETS
tof_ros2cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

## Copying config files and depth compute libraries from sdk/config, sdk/lib folders to build/examples/first-frame and build/first-frame/Debug or Release folders
FILE(GLOB CONFIG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/tof_config/*)
install(FILES ${CONFIG_FILES} DESTINATION lib/../../../config/)
install(FILES ${CONFIG_FILES} DESTINATION lib/${PROJECT_NAME}/config/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()

