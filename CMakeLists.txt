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

# for debug tof in qt creator
###find_package(aditof PATHS "/home/vholonec/work/build-ToF-Desktop-Debug/sdk" REQUIRED)
#include_directories("/home/vholonec/work/build-ToF-Desktop-Debug/sdk")
#link_directories("/home/vholonec/work/build-ToF-Desktop-Debug/sdk/")
#add_library(aditof SHARED IMPORTED) # or STATIC instead of SHARED
#set_target_properties(aditof PROPERTIES
#  IMPORTED_LOCATION "/home/vholonec/work/build-ToF-Desktop-Debug/sdk/libaditof.so"
#  INTERFACE_INCLUDE_DIRECTORIES "/home/vholonec/work/build-ToF-Desktop-Debug/sdk"
#)
#add_library(tofi_compute SHARED IMPORTED) # or STATIC instead of SHARED
#set_target_properties(tofi_compute PROPERTIES
#  IMPORTED_LOCATION "/home/vholonec/work/build-ToF-Desktop-Debug/sdk/common/adi/depth-compute-opensource/libtofi_compute.so"
#  INTERFACE_INCLUDE_DIRECTORIES "/home/vholonec/work/build-ToF-Desktop-Debug/sdk/common/adi/depth-compute-opensource"
#)
#add_library(tofi_config SHARED IMPORTED) # or STATIC instead of SHARED
#set_target_properties(tofi_config PROPERTIES
#  IMPORTED_LOCATION "/home/vholonec/work/build-ToF-Desktop-Debug/sdk/common/adi/depth-compute-opensource/libtofi_config.so"
#  INTERFACE_INCLUDE_DIRECTORIES "/home/vholonec/work/build-ToF-Desktop-Debug/sdk/common/adi/depth-compute-opensource"
#)

find_library(TOFI_COMPUTE
  NAMES tofi_compute
  PATHS /usr/lib)

set(SOURCE ${CMAKE_SOURCE_DIR}/include/tof-ros2)

#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../tof_ros2_binary/install/)

set(SOURCES ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)
FILE(GLOB_RECURSE LibFiles ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp/*.h)

file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)


add_executable(tof_ros2cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_camera_node.cpp ${SOURCES} ${LibFiles})


target_include_directories(tof_ros2cpp PUBLIC ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)
target_sources(tof_ros2cpp INTERFACE ${CMAKE_SOURCE_DIR}/include/tof_ros2cpp)

ament_target_dependencies(tof_ros2cpp image_transport rclcpp std_msgs glog)

target_include_directories(tof_ros2cpp PUBLIC "${ADITOF_INCLUDE_DIRS}")
target_include_directories(tof_ros2cpp PUBLIC "${LIBWEBSOCKETS_INCLUDE_DIRS}")
target_link_libraries(tof_ros2cpp "${ADITOF_LIBRARIES}")
target_link_libraries(tof_ros2cpp aditof)
target_link_libraries(tof_ros2cpp websockets_shared)
target_link_libraries(tof_ros2cpp tofi_compute)
target_link_libraries(tof_ros2cpp tofi_config)
target_link_libraries(tof_ros2cpp glog::glog)

install(TARGETS
tof_ros2cpp
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

ament_package()

