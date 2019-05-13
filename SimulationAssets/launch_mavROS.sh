#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
source /home/oschofield/srcMasters/sitlWS/devel/setup.bash

roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost@14557"
