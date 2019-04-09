Drones4Energy - Approximation and Guided Landing on Powerlines MSc Thesis
University of Southern Denmark

# Dependencies
For the Project to work the following dependencies must be installed
```bash
sudo apt-get install ros-kinetic-cv-bridge
sudo apt-get install ros-kinetic-video-stream-opencv
```

The Robwork Libary is also Nessesary to run the system. No need to install all the optinal dependecies and RWSim RWHardvare and RWStudio

# Starting the Project

To Run the Algorithms you must first have a flight Recording from Airsim and change the path to the flight recording in Image_aquisition/src/airsim_images_main.cpp

```bash
roslaunch image_aquisition airsim_img.launch
roslaunch plined prog.launch
roslaunch estimator3d prog.launch
roslaunch drone_motion prog.launch
```


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

