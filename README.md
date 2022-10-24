# ROS2 Wrapper for [Time of Flight SDK](https://https://github.com/analogdevicesinc/ToF) of Analog Devices (for Ubuntu)


# 1. Install ROS2

- Install the recommended [ROS2 distribution](https://docs.ros.org/en/rolling/Releases.html) for your operating system**
  - [ROS Install page](https://docs.ros.org/en/foxy/Installation.html)

- In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following libraries:

# 2. ToF dependency

In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following library:


- [Build ToF from sourcecode](https://github.com/analogdevicesinc/ToF/blob/master/doc/itof/linux_build_instructions.md) (Until the step: "Download and build the SDK only")

Also make sure to run ```sudo make install``` at the end of the build

# 3. Usage

In directory ```ros2_ws/src/``` clone the repository:

```console
git clone https://github.com/rbudai98/tof_ros2cpp.git
```

After cloning the repository in the ``ros2_ws/ run the following command:
 
```console
colcon build
source devel/setup.bash
```

### Starting camera node
- In the general ROS2 workspace run the following code:
```console
    ros2 run tof_ros2cpp tof_camera_node ip="10.42.0.1" config_file="config/config_walden_3500_nxp.json" use_depthCompute="true" mode=1
```
### Parameters
 [config_file:"<<b>path></b>"]
* Crosby with Pulsatrix: "```config/config_walden_3500_nxp.json```"
* Crosby: "```config/config_crosby_nxp.json```"
* Walden: "```config/config_walden_nxp.json```"

 [use_depthCompute] 
 - "true" for enabling Depth Compute libraries
 - "false" for disabling Depth Compute libraries 

 [mode]:
* 1 -> LR - QMP mode of the camera
* 2 -> LR - MP mode of the camera

