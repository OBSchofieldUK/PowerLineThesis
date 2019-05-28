# Simulation install and configuration

To run the PowerLineThesis code within a simulation environment, you will need to set up a PX4 Gazebo environment

1. Configure your computer using the PX4 Bash Scripts located [here](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh)


1a. If running on Ubuntu 18.04 you will need to change 'kinetic' to 'melodic'

NB: This will be the same for all scripts in this readme 

2. PX4 firware needs to be v1.8.2 (latest stable release)
3. Test that gazebo/PX4 Simulation works

use the commands below to ensure the 
    ```Bash
    git clone https://github.com/PX4/Firmware
    cd Firmware
    git checkout v1.8.2
    make posix_sitl_default gazebo
    ```

If everything builds and is configured correctly, a Gazebo window should appear with a Iris drone. 
use the command `commander takeoff` within the terminal window to launch the drone.

# Install Simulation Assets
run the command `./install_Assets.sh add/path/to/Firmware/folder` to install the simulation assets to your Firmware folder.

#running simulation
use ` ./px4_launch_gazebo.sh` to launch the simulation environment.
use `./launch_mavROS.sh` to connect mavros to the simuation environment.

To use the droneControl framework, look at the readme in the droneControl Folder