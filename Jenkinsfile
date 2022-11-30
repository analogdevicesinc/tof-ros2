pipeline {
    agent { docker { image 'ubuntu20.04/ros2_foxy'
                    args '-u root:sudo' } }
      stages {

        stage ('ToF SDK setup')
        {
            steps{
                sh 'rm -rf ToF && git clone https://github.com/analogdevicesinc/ToF && cd ToF && mkdir build && cd build && cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=1 .. &&  make install .'
            }
        }
        stage('Clone tof-ros2 and colcon build')
         {
            steps {
                sh 'mkdir -p ros2_ws/src && rm -rf tof-ros2 && git clone https://github.com/analogdevicesinc/tof-ros2'
                sh 'source /opt/ros/foxy/setup.bash && cd ros2_ws/src && colcon build'
            }
    }
  }
}