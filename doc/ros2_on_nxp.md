# Install tof-ros2 project on NXP camera with ToF

First install an ubuntu image on a sdcard.
You can take lastest ubuntu image with ToF project from url

[https://github.com/analogdevicesinc/ToF/releases](https://github.com/analogdevicesinc/ToF/releases)

After you login on NXP camera with ubuntu image connect camera
on network and with usb cable to pc.
[https://wiki.analog.com/eval-adtf3175-nxz-startup](https://wiki.analog.com/eval-adtf3175-nxz-startup)

Update ToF build to latest release version.
You can install ToF project from here. 
[https://github.com/analogdevicesinc/ToF/blob/main/doc/itof/linux_build_instructions.md](https://github.com/analogdevicesinc/ToF/blob/main/doc/itof/linux_build_instructions.md)

Now you can install ros2 on nxp camera.
The archive with source code of ros2 framework will be written here in this doc.
It will be on an server and you can get ros2 archive with wget command.

For building ros2 you need colcon build tool.
You can install colcon from 
[https://colcon.readthedocs.io/en/released/user/installation.html](https://colcon.readthedocs.io/en/released/user/installation.html)
For installing colcon you may install curl package and its dependency gnupg package

After install colcon make a directory in home folder

```console
  mkdir ros2_foxy
```

Download the ros2 archive with the name scr in ros2_foxy folder.

Extract the archive with the tar command

You will have something like ~/ros2_foxy/src

Now in src folder clone the tof-ros2 project

```console
  git clone https://github.com/analogdevicesinc/tof-ros2.git
```

Then install some ros2 library dependencies

```console
  sudo apt-get install libasio-dev
  sudo apt install libtinyxml2-dev
  sudo apt-get install -y liblog4cxx-dev
  pip3 install lark
```

Now cd in folder ~/ros2_foxy folder

Build the tof-ros2 project and ros2 archive

```console
  colcon build --packages-skip-build-finished --packages-up-to tof_ros2cpp
```

This will install ros2 and tof-ros2 over 100 ros2 packages
Wait for building the project about one , two hours.

After install the tof-ros2 now you can run the node tof-ros2
To run the node you must install another package.

```console
  colcon build --packages-skip-build-finished --packages-up-to ros2run
```

For run the tof_ros2 project you must run before two commands source and export
In folder ~/ros2_foxy you run the comands

```console
  source install/local_setup.sh
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH;/opt/websockets/lib;/usr/local/lib"
```

Now you can run the node tof-ros2

```console
  ros2 run tof_ros2cpp tof_ros2cpp_node config_file=config/config_adsd3500_adsd3100.json mode=1 enable_multithread=true
```

