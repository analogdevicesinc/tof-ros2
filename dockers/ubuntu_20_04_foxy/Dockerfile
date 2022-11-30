FROM ubuntu:20.04
 
LABEL maintainer="Robert Budai, <robert.budai@analog.com>"
ARG DEBIAN_FRONTEND=noninteractive

# Install updates to base image
RUN \
  apt-get update -y \
  && apt-get install -y git \
  && apt-get install -y cmake \
  && apt-get install -y build-essential \
  && apt-get install apt-utils

#Install Glog
RUN \
  git clone -b v0.6.0 --depth 1 https://github.com/google/glog \
  && cd glog \
  && mkdir build_0_6_0 && cd build_0_6_0 \
  && cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog .. \
  && cmake --build . --target install

#Install Libwebsoocket
RUN \
  git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets \
  && cd libwebsockets \
  && mkdir build_3_1 && cd build_3_1 \
  && cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets .. \
  && cmake --build . --target install

#Install Protobuf
RUN \
  git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf \
  && cd protobuf \
  && mkdir build_3_9_0 && cd build_3_9_0 \
  && cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake \
  && cmake --build . --target install

# #Install ToF SDK
# RUN \
#   git clone https://github.com/analogdevicesinc/ToF \
#   && cd ToF \
#   && mkdir build && cd build \
#   && cmake -DUSE_DEPTH_COMPUTE_OPENSOURCE=1 -DWITH_NETWORK=1 -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" .. \
#   && cmake --build . --target install 

#Install ROS2
RUN \
  locale \
  && apt-get update && apt-get install locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8 
RUN \
  apt-get install -y software-properties-common \
  && add-apt-repository universe \
  && apt-get update \
  && apt-get install -y curl 
RUN \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update \
  && apt-get upgrade -y
RUN \
  apt-get install -y ros-foxy-desktop python3-argcomplete \
  && apt-get install -y python3-colcon-common-extensions

RUN \
  mkdir ros2_ws && cd ros2_ws && mkdir src