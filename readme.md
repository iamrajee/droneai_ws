# Test ROS GAZEBO GUI

## sudo apt install docker.io
## docker pull osrf/ros:noetic-desktop-full
## xhost +
## docker run -it --net=host     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     osrf/ros:noetic-desktop-full     bash -it -c "roslaunch gazebo_ros empty_world.launch"


# Setup
## Terminal 1
### Sudo docker run -it ros:noetic-desktop-full
### ./ros_entrypoint.sh
### git clone https://github.com/iamrajee/droneai_ws
### cd droneai_ws
### chmod +x *.sh
### catkin_make
### ./run.sh

## Terminal 2
### docker exec -it [docker_unique_name] bash
### ./ros_entrypoint.sh
### rostopic list -v

