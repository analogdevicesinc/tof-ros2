pipeline {
    agent { docker { image 'ubuntu20.04/ros2_foxy' } }
      stages {
        stage('log version info') {
      steps {
        sh 'cd ~ && git clone https://github.com/analogdevicesinc/ToF && cd ToF && mkdir build && cd build && cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=1 .. && make install .'
        sh 'cd ~/ros2_ws/src && git clone https://github.com/analogdevicesinc/tof-ros2'
        sh 'source /optros/foxy/setup.bash && cd ~/ros2_ws/src && colcon build'
      }
    }
  }
}