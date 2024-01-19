## Using [DPKG packages](https://github.com/analogdevicesinc/tof-ros2?tab=readme-ov-file#download-debian-package)

These dockers are intended to prepare the tof dpkg packages for easy library setup for users.
The generation can be done using the ```docker_build.sh```, by running:
 ```console
 bash docker_build.sh
 ```
After the build is completed you can find the ```.dpkg``` tof dependency library files for tof-ros2

Installing the libraries:
```console
sudo dpkg -i tof_lib.deb