Drones4Energy - Approximation and Guided Landing on Powerlines MSc Thesis
University of Southern Denmark

# Dependencies
For the Project to work the following dependencies must be installed
```bash
sudo apt-get install ros-kinetic-cv-bridge
sudo apt-get install ros-kinetic-video-stream-opencv
```
The Rapidjson libary is nessesary to run the code. It is currently tested with release version 1.1.0. If [-Werror=implicit-fallthrough=] ocurs during build go into the file and write ```continue;``` at the end of the cases.

The Eigen 3 libary is nessesary for building the project

To run the Project open cv is also nessesary. Currently it is tested with Opencv 3.3.1.

The Project also depends on ROS. The implementation is tested to work on ros-kinetic, and ros-melodic.

# Starting the Project

When the Nodes are run for the first time a setting.json file is generated in the main directory where settings souch as the location of the Airsim recording folder can be found.

```bash
roslaunch image_aquisition airsim_img.launch        #Video From Airsim Recording
 or
roslaunch image_aquisition Webcam.launch video:=0   #Video from connected camera

roslaunch image_processing prog.launch              #Find PowerLines
roslaunch estimator3d prog.launch                   #Estimate 3D position of powerLines
roslaunch drone_motion prog.launch                  #Calculate Relative Motion
roslaunch powerline_selector prog.launch            #OutPut data of the data aquisition system
roslaunch lidar_matcher Lidar.launch                #Starts Lidar

```
# Rostopics
The Rostopics used in the project 

```bash
/DroneInfo/Position             #Sends the Position of the drone global
/DroneInfo/Relative/Position    #Sends the Relative Motion of the Drone camera since last image
/Estimator/lines2d              #Sends the Predicted position of the powerline in next image
/Estimator/lines3d              #Sends the result of the 3d estimation of powerlines for good estimates
/linedetector/lines2d           #Sends the 2d lines detected from the Image (Must be matached with id from estimater)
/webcam/image_raw               #Sends the Image
```

# PowerLine Simulation 
Simulation Assets have been configured for use with PX4/Gazebo for debugging. To install, look under the Simulation Assets Folder. 

# Copyright

Copyright (c) 2018


Oscar Bowen Schofield (osbow17@student.sdu.dk) \
Kasper Høj Lorenzen  (kalor14@student.sdu.dk)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

