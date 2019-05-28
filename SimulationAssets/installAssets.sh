#!/bin/bash

FIRMDIR=$1
CWD=$PWD
if [[ "$FIRMDIR" == "" ]]; then
  echo "You did not supply the Firmware folder."
  exit
fi
echo "installing launch file"
cd $FIRMDIR
cp $CWD/msc_posix.launch launch/ -f
cp $CWD/msc_posix_lidar.launch launch/ -f

echo "installing irislidar_msc models"
cd Tools/sitl_gazebo/models
cp $CWD/irislidar_msc . -f -r
cp $CWD/iris_msc . -f -r

echo "installing world."
cd ../worlds
cp $CWD/msc_world.world . -f -r

echo "done."
