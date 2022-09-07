# include(ExternalProject)

# ExternalProject_Add(aditof
#     GIT_REPOSITORY "https://github.com/analogdevicesinc/ToF.git"
#     GIT_TAG update_libwebsocket_linking
#     SOURCE_DIR "${CMAKE_BINARY_DIR}/ToF"
#     PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ToF
#     BINARY_DIR build
#     CMAKE_CACHE_ARGS
#         -DADITOF_ENABLE_BAR:BOOL=1
#         -DWITH_NETWORK:BOOL=1 
#         -DWITH_ROS:BOOL=0
#         -DWITH_EXAMPLES:BOOL=0
#         -DCMAKE_PREFIX_PATH:STRING=/opt/glog;/opt/protobuf;/opt/websockets
# )


# # ExternalProject_Add(tof_ros2cpp
# #   SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
# #   DOWNLOAD_COMMAND ""
# #   UPDATE_COMMAND ""
# #   CMAKE_CACHE_ARGS
# #     -aditof_DIR:PATH=${CMAKE_BINARY_DIR}/ToF/build
# #     -Dtof_ros2cpp_ENABLE_SDKBUILD:BOOL=${tof_ros2cpp_ENABLE_SDKBUILD}
# #   INSTALL_COMMAND ""
# #   )   