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
* Tof lib for ([Ubuntu 20.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/crosby/out_ubuntu20/tof_lib.deb)) [Latest SDK master]
* Tof lib for ([Ubuntu 22.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/crosby/out_ubuntu22/tof_lib.deb)) [Latest SDK master]

#### For Adsd3030:
* Tof lib for ([Ubuntu 20.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/adsd3030/out_ubuntu20/tof_lib.deb)) [Latest SDK master]
* Tof lib for ([Ubuntu 22.04](https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/adsd3030/out_ubuntu22/tof_lib.deb)) [Latest SDK master]

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
- In the general ROS2 workspace run the following code, setting up the path towards shaed library:

```console
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH;/opt/websockets/lib;/usr/local/lib"
```
### With ```roslaunch```

* EVAL-ADTF3175D : 
```console
ros2 launch tof_ros2cpp EVAL-ADTF3175D.launch.xml
```

### With ```ros2 run```

- Starting the node
```console
  ros2 run tof_ros2cpp tof_ros2cpp_node ip=10.42.0.1 config_file=<config file path> mode=<mode number> enable_multithread=<true/false>
```

#### Parameters:
[ip = "<<b>ip address of camera></b>"]
* for remote acces specify the cameras ip: ```10.42.0.1``` 
* for on target use, leave empty: ```""``` 

 Default value: ```""``` (empty string)


 [config_file = "<<b>config file path></b>"]
* ```config/config_adsd3500_adsd3100.json``` ("Crosby")
* ```config/config_adsd3500_adsd3030.json``` ("Adsd3030")

Default value: ```config/config_adsd3500_adsd3100.json```

 [mode = "<<b>mode></b>"] (for both cameras):
 * 0: sr-native (short-range native)
 * 1: lr-native (long-range native)
 * 2: sr-qnative (short-range quarter native)
 * 3: lr-qnative (long-range quarter native)
 * 4: pcm-native 
 * 5: sr-mixed (short-range mixed)
 * 6: lr-mixed (long-range mixed)

Default value: ```0```

 [enable_multithread = "<<b>True/False</b>>"]:
 * True: Node creates different threads for each publishing topics
 * False: All publishers are updated on the same thread
 
Default value: ```False```

Note: Although multithreading provides a faster publishing rate on certain platforms, on less performant Hosts this might not be beneficial and the single thread implementation can be more relevant.

## ROS Parameters
### Camera parameters:
* ```adsd3500ABinvalidationThreshold``` 
* ```adsd3500ConfidenceThreshold``` 
* ```adsd3500JBLFfilterEnableStat``` 
* ```adsd3500JBLFfilterSize``` 
* ```adsd3500RadialThresholdMin``` 
* ```adsd3500RadialThresholdMax``` 

### Thread parameters:
* ```ir```
* ```depth```
* ```raw```
* ```conf```
* ```xyz```

To enable or disable a publisher thread you can use ros2 param set commnad:

```console
  ros2 param set /tof_camera_node depth false
```

The thread parameter name are: ir, depth, raw, conf and xyz.
The thread parameter type are bool: true, false.
Info: You must run twice the ros2 param set commnad to have effect on the node.
