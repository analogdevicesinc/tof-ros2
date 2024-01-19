# Dockers for tof and tof-ros2

## Running Docker files:
Choose the appropriate ubuntu docker from the available list:
* Ubuntu 20.04 ROS2 Foxy
* Ubuntu 20.04 ROS2 Galactic
* Ubuntu 22.04 ROS2 Humble

### Step 1:
Build the Docker file with:
```console
sudo docker build --tag <build_tag> .
```
Or build from skratch:
```console
sudo docker build --no-cache --tag <build_tag> .
``
### Step 2:
Run Docker image in container interactively:
```console
sudo docker run -it <docker_image_name>
```