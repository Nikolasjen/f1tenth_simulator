## This repo is based on the F1TENTH Racecar Simulator by 
###@kimnluong, kimnluong Kim Luong
###@hzheng40, hzheng40 Hongrui (Billy) Zheng
## See: https://github.com/f1tenth/f1tenth_simulator
### Please refer to their repo for greater detail on the F1tenth simulator 

# The following is an addaption of @kimnluong and @hzheng40's installation guide to work with this repo
## Dependencies

Please first install ```ros-noetic-desktop```, afterwards you can install the additional dependencies:

- tf2_geometry_msgs
- ackermann_msgs
- map_server

These can be installed by running:

    sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-map-server

Similar to their project, the full list of dependencies can be found in the ```package.xml``` file.

### Installation

To install the simulator package, clone the repo with the simulator and starter code into your catkin workspace:

    cd ~/catkin_ws/src
    git clone https://github.com/Nikolasjen/f1tenth_simulator.git
    
Then run ```catkin_make``` to build it:

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

In case this does not work, please try with ```catkin_make --force-cmake``` as this will forcefully load the whole catkin project

## Quick Start

To run the simulator without the evolutionary algorithm, run:
    
    cd ~
    rosrun f1tenth_simulator test_simulator.py

This will launch everything you need for a full simulation; roscore, the simulator, a preselected map, a model of the racecar and RViz.

If you instead want to run the evolutionary algorithm, I suggest looking at the ```POPULATION_SIZE```, the ```MAX_RUNNING``` and the ```NUMBER_OF_GENERATIONS``` parameters as these are in charge of how many how many simulations will run per generation, how many simultanious simulations will run (check what your system can handle) and how many generations will run, respectively. Remember that one simulation will not finish until the racecar either finishes a lap or collides with a wall.

Then you can run the script via:
    cd ~
    rosrun f1tenth_simulator my_simulator.py
