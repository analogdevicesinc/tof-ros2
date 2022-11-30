pipeline {
    agent none
      stages {
        stage ('Ubuntu 20.04/ ROS2 Foxy')
        {
            agent { 
                docker { 
                    image 'ubuntu20.04/ros2_foxy'
                    args '-u root:sudo'
                        } 
                }
            steps{
                sh '''
                #!/bin/bash
                rm -rf ToF && git clone https://github.com/analogdevicesinc/ToF && cd ToF && mkdir build && cd build && cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=1 .. &&  make install .
                '''
           
                sh '''
                #!/bin/bash
                rm -rf ros2_ws && mkdir -p ros2_ws/src && cd ros2_ws/src && git clone https://github.com/analogdevicesinc/tof-ros2 && cd ../ && . /opt/ros/foxy/setup.sh && colcon build
                '''
            }
        }

        stage ('Ubuntu 20.04/ ROS2 Galactic')
        {
            agent { 
                docker { 
                    image 'ubuntu20.04/ros2_galactic'
                    args '-u root:sudo'
                        } 
                }
            steps{
                sh '''
                #!/bin/bash
                rm -rf ToF && git clone https://github.com/analogdevicesinc/ToF && cd ToF && mkdir build && cd build && cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=1 .. &&  make install .
                '''
           
                sh '''
                #!/bin/bash
                rm -rf ros2_ws && mkdir -p ros2_ws/src && cd ros2_ws/src && git clone https://github.com/analogdevicesinc/tof-ros2 && cd ../ && . /opt/ros/galactic/setup.sh && colcon build
                '''
            }
        }

        stage ('Ubuntu 22.04/ ROS2 Humble')
        {
            agent { 
                docker { 
                    image 'ubuntu22.04/ros2_humble'
                    args '-u root:sudo'
                        } 
                }
            steps{
                sh '''
                #!/bin/bash
                rm -rf ToF && git clone https://github.com/analogdevicesinc/ToF && cd ToF && mkdir build && cd build && cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=1 .. &&  make install .
                '''
           
                sh '''
                #!/bin/bash
                rm -rf ros2_ws && mkdir -p ros2_ws/src && cd ros2_ws/src && git clone https://github.com/analogdevicesinc/tof-ros2 && cd ../ && . /opt/ros/humble/setup.sh && colcon build
                '''
            }
        }
  }
}