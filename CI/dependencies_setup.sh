#!/bin/bash

# Base Image Updates
sudo apt-get update -y
sudo apt-get install -y git cmake build-essential apt-utils libssl-dev wget

# Install cmake v3.20
wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz
tar -zvxf cmake-3.20.0.tar.gz
cd cmake-3.20.0
./bootstrap
sudo make -j8
sudo make install .

# Install Glog
git clone -b v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir -p build_0_6_0 && cd build_0_6_0
sudo cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install

# Install Libwebsocket
sudo apt-get install libssl-dev
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
sudo cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install

# Install Protobuf
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
sudo cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install

# Install OpenGL
sudo apt install libgl1-mesa-dev libglfw3-dev -y
