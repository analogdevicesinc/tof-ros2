# ROS2 Wrapper for [Time of Flight SDK](https://github.com/analogdevicesinc/ToF) of Analog Devices (for Ubuntu)

|  Ubuntu 20.04: Foxy |  Ubuntu 22.04: Humble | Ubuntu 22.04: Rolling  |
|---|---|---|
| [![Build Status](https://dev.azure.com/AnalogDevices/ToF/_apis/build/status%2Fanalogdevicesinc.tof-ros2?branchName=main&jobName=Job&configuration=Job%20ubuntu_20_04_foxy)](https://dev.azure.com/AnalogDevices/ToF/_build/latest?definitionId=46&branchName=main)  |  [![Build Status](https://dev.azure.com/AnalogDevices/ToF/_apis/build/status%2Fanalogdevicesinc.tof-ros2?branchName=main&jobName=Job&configuration=Job%20ubuntu_22_04_humble)](https://dev.azure.com/AnalogDevices/ToF/_build/latest?definitionId=46&branchName=main)  |  [![Build Status](https://dev.azure.com/AnalogDevices/ToF/_apis/build/status%2Fanalogdevicesinc.tof-ros2?branchName=main&jobName=Job&configuration=Job%20ubuntu_22_04_rolling)](https://dev.azure.com/AnalogDevices/ToF/_build/latest?definitionId=46&branchName=main) |

## 1. Install ROS2

- Install the recommended [ROS2 distribution](https://docs.ros.org/en/rolling/Releases.html) for your operating system**
  - [ROS Install page](https://docs.ros.org/en/foxy/Installation.html)

- In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following libraries:

## 2. ToF dependency
### Download debian package:
#### For Crosby:
* Tof lib for ([Ubuntu 18.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/crosby/out_ubuntu18/tof_lib.deb)) [Deprecated]
* Tof lib for ([Ubuntu 20.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/crosby/out_ubuntu20/tof_lib.deb)) [Rel. ver. 4.2.0]
* Tof lib for ([Ubuntu 22.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/crosby/out_ubuntu22/tof_lib.deb)) [Rel. ver. 4.2.0]

#### For Adsd3030:
* Tof lib for ([Ubuntu 18.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/adsd3030/out_ubuntu18/tof_lib.deb)) [Deprecated]
* Tof lib for ([Ubuntu 20.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/adsd3030/out_ubuntu20/tof_lib.deb)) [Rel. ver. 4.2.0]
* Tof lib for ([Ubuntu 22.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/adsd3030/out_ubuntu22/tof_lib.deb)) [Rel. ver. 4.2.0]

Install command: ```sudo dpkg -i tof_lib.deb```
### Building from sources
In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following library:

- [Build ToF from sourcecode](https://github.com/analogdevicesinc/ToF/blob/master/doc/itof/linux_build_instructions.md) (Until the step: "Download and build the SDK only")

Also make sure to run ```sudo make install``` at the end of the build

## 3. Usage

In directory ```ros2_ws/src/``` clone the repository:

```console
  git clone https://github.com/analogdevicesinc/tof-ros2
```

After cloning the repository in the ``ros2_ws/ run the following command:
 
```console
  colcon build
  source install/setup.sh
```

## Starting camera node

### With ```roslaunch```

* EVAL-ADTF3175D : ```ros2 launch tof_ros2cpp EVAL-ADTF3175D.launch.xml```

### With ```ros2 run```
- In the general ROS2 workspace run the following code, setting up the path towards shaed library:
```console
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH;/opt/websockets/lib;/usr/local/lib"
```
- Starting the node
```console
  ros2 run tof_ros2cpp tof_ros2cpp ip=10.42.0.1 config_file=<config file path> mode=<mode number>
```

#### Parameters:
 [config_file = "<<b>config file path></b>"]
* ```config/config_adsd3500_adsd3100.json``` ("Crosby")
* ```config/config_adsd3500_adsd3030.json``` ("Adsd3030")


 [mode = "<<b>mode></b>"] (for both cameras):
 * 0: sr-native (short-range native)
 * 1: lr-native (long-range native)
 * 2: sr-qnative (short-range quarter native)
 * 3: lr-qnative (long-range quarter native)
 * 4: pcm-native 
 * 6: sr-mixed (short-range mixed)
 * 5: lr-mixed (long-range mixed)

