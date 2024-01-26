# Test ROS GAZEBO GUI

## sudo apt install docker.io
## docker pull osrf/ros:noetic-desktop-full
## xhost +
## docker run -it --net=host     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     osrf/ros:noetic-desktop-full     bash -it -c "roslaunch gazebo_ros empty_world.launch"


# Setup
## Terminal 1
### sudo docker run -it osrf/ros:noetic-desktop-full
### source ros_entrypoint.sh
### git clone https://github.com/iamrajee/droneai_ws
### cd droneai_ws
### chmod +x *.sh
### catkin_make
### ./run.sh

## Terminal 2
### docker exec -it [docker_unique_name] bash
### source ros_entrypoint.sh
### rostopic list -v

xhost +local:root
docker run -it --privileged --net=host --env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/dev/dri:/dev/dri" osrf/ros:noetic-desktop-full bash -it -c "roslaunch gazebo_ros empty_world.launch"

export DISPLAY=:0

sudo docker run -it --privileged --net=host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:noetic-desktop-full bash
docker exec -it lucid_feistel bash

apt-get install mesa-utils
glxgears