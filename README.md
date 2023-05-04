# Autonomus-Homing-in-UUV 

First, one needs to install the uuv-simulator and the necessary plugins for gazebo on ROS Melodic. This can be done simply by cloning the github repository of uuv_simulator. 
The ```uuv_tutorial_dp_controller``` package in this repository contains the ```start_tutorial_dp_controller_demo.launch``` file which is modified according to the requirements of the code to be run. One should run this file, in the terminal after sourcing it. <br>
Note - this file is a modiffied version of the one that is already present in the package. Hence one needs to replace this one or change the package name and build it in the catkin workspace before running it. <br>
Next, the ```uuv_new_controller``` package contains the ```controller.py``` which needs to be tun using ```rosrun```. This should be done after sourcing the workspace in the terminal. 
The results should be seen in RViz! <br>
Good Luck :) 

# Waypoint Following 
As done in the previous case, one again needs to run the ```start_tutorial_dp_controller_demo.launch``` launch file which has the ```.world``` file embedded in it. Next, instead of running ```controller.py``` we need to run ```controller_2.py``` by making editions for the desired waypoints to be followed. Only the co-ordinates of the goal positions and the waypoints list need to be changed. Then run and visulaize the movement in RViz. <br> 

# SONAR scanning 
The various world files have been included in the ```worlds and models``` folder. One needs to change the world file - by adding the right models (for wall blocks) and also change the ```launch``` file - to include the right worlds (```lake``` or ```empty underwater world``` or ```ocean world```), and then run the simulation as done previously. Moreover, if one wants to allow the vehicle to hover and rise up to the surface without following any waypoints or any thrust from the thrusters, one needs to uncomment lines - ```198-202``` from ```controller_2.py``` and comment out lines ```203-287``` and then run the code. 
In order to visualize the scanned points in RViz - one needs to add the ```laserscan``` topic in RViz from the 'by topic' option and increase the size of the scans to be able to attain a clear scan imagery. 
