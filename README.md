phantomx_rst ROS Package
========================

This metapackage is intended for controlling and simulating the PhantomX Pincher robot at the RST.
It is developed and composed for educational purposes and contains relevant packages.


Installation
------------

First, make sure that ROS and moveit is installed properly (we are currently on ROS indigo):
    
    sudo apt-get install ros-indigo-moveit-full

The arbotix package is required since it constitutes a low-level driver package that interfaces with the robot hardware.
But an important feature is missing in the official version, thus we are using a modified one (until changes are accepted in the original verion)

    cd ~/catkin_ws/src
    git clone https://github.com/croesmann/arbotix_ros.git


Now it is time to checkout this package:

    git clone https://github.com/rst-tu-dortmund/phantomx_rst.git


Check if everything compiles:

    cd ~/catkin-ws
    catkin_make
    

Getting Started
---------------

**Arbotix Controllers**

The controller is required for all applications:
    
    roslaunch phantomx_rst arm.launch

In case of a simulation launch:
 
    roslaunch phantomx_rst arm_sim.launch


**MoveIt (GUI)**

Run

    roslaunch phantomx_rst arm_moveit.launch


**MoveIt (C++)**

Close the GUI and start visualization only

    roslaunch phantomx_rst moveit_rviz.launch


Start the move_group instance (required for planning)
 
    roslaunch phantomx_rst move_group.launch


Look at the code of the *phantomx_control moveit node* and feel free to insert your own code here.
Run that node using

    rosrun phantomx_control moveit_control


**Simple PhantomX API**

Look at the phantomx_lib Code-API.
More information will be provided here soon...

    rosrun phantomx_control simple_control


Remarks
-------

Note, this package includes an (slighly) adopted version of the turtlebot_arm package (resp. the branch https://github.com/corot/turtlebot_arm.git).
It is copied into this metapackage in order to allow a simple integration for students that start working with the PhantomX.
