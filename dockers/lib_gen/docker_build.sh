#Building docker files from source
echo "Building dockers"

#Ubuntu 20
sudo docker build --tag ubuntu20.04/ros2_foxy ./ubuntu_20_04_foxy/
sudo docker build --tag ubuntu20.04/ros2_galactic ./ubuntu_20_04_galactic/
sudo docker build --tag ubuntu20.04/ros2_humble ./ubuntu_20_04_humble/

#Ubuntu 22
sudo docker build --tag ubuntu22.04/ros2_humble ./ubuntu_22_04_humble/
