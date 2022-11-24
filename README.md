# Autonomus-Homing-in-UUV 

First, one needs to install the uuv-simulator and the necessary plugins for gazebo on ROS Melodic. This can be done simply by cloning the github repository of uuv_simulator. 
The ```uuv_tutorial_dp_controller``` package in this repository contains the ```start_tutorial_dp_controller_demo.launch``` file which is modified according to the requirements of the code to be run. One should run this file, in the terminal after sourcing it. <br>
Note - this file is a modiffied version of the one that is already present in the package. Hence one needs to replace this one or change the package name and build it in the catkin workspace before running it. <br>
Next, the ```uuv_new_controller``` package contains the ```controller.py``` which needs to be tun using ```rosrun```. This should be done after sourcing the workspace in the terminal. 
The results should be seen in RViz! <br>
Good Luck :)
