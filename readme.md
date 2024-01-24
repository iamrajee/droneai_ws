# Setup

## sudo apt install docker.io
## docker pull osrf/ros:noetic-desktop-full
## docker run -it --net=host     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     osrf/ros:noetic-desktop-full     bash -it -c "roslaunch gazebo_ros empty_world.launch"
## xhost +
