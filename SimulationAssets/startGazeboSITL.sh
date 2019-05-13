#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
source /home/oschofield/srcMasters/sitlWS/devel/setup.bash

export PX4_HOME_LAT=55.43620
export PX4_HOME_LON=10.46091

# roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost@14557" &

if [[ $1 = "" ]]; then
  FIRMDIR="/home/oschofield/srcMasters/Firmware/"
else
  FIRMDIR=$1
fi

# argument used to browse to your PX4 SITL firmware folder
cd $FIRMDIR


# Needed environment for running SITL
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# roslaunch px4 gui:=true posix_sitl.launch
make px4_sitl_default gazebo

# roslaunch px4 MSC_posix_sitl.launch

# # # create 2 linked virtual serial ports
# socat -d -d pty,raw,echo=0,link=/tmp/sim1 pty,raw,echo=0,link=/tmp/sim2 &
#
# # # Bridge serial-udp
# socat -d udp4-listen:14540 open:/tmp/sim1,raw,nonblock,waitlock=/tmp/s0.locak,echo=0,b115200,crnl &
