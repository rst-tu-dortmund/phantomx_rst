phantomx_rst ROS Package
========================

This metapackage is intended for controlling and simulating the PhantomX Pincher robot at the RST.

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
    
    sudo chmod 777 /dev/ttyUSB0   # Set permissions for the usb/serial conv.
    roslaunch phantomx_rst arm.launch

In case of a simulation launch:
 
    roslaunch phantomx_rst arm_sim.launch


**MoveIt (GUI)**

Run

    roslaunch moveit_gui_all.launch


**MoveIt (C++)**

Close the GUI and start visualization only

    roslaunch phantomx_rst arm_rviz.launch


Start the move_group instance (required for planning)
 
    roslaunch phantomx_rst move_group.launch


Look at the code of the *phantomx_control moveit node* and feel free to insert your own code here.
Run that node using

    rosrun phantomx_control moveit_control


**Simple PhantomX API**

Look at the phantomx_lib Code-API.
More information will be provided here soon...

    roslaunch phantomx_rst arm_rviz.launch
    rosrun phantomx_control simple_control


**Utilities**

Print joint values for checking servos. Verify that the zero position and all angle limits are correct.

    rosrun phantomx_rst print_joint_state.py

Relax servos and print end-effector pose (requires that the default position of the robot is correct: q=0 -> upright arm position)

    rosrun phantomx_control measure_states


Licence
-------
The *phantomx_rst* meta-package is mainly developed and composed for educational purposes.

The *phantomx_lib*, *phantomx_control* and *phantomx_rst* packages (as part of the meta-package) are licensed under the BSD license.
The packages depend on other ROS packages, which are listed in the package.xml and that are also BSD licensed,
and the following third-party packages:
 * Eigen, MPL2 license, http://eigen.tuxfamily.org
 * OpenCV, BSD license, http://opencv.org

In order to simplify the configuration process, especially for educational purposes,
some modified versions of the *turtlebot_arm* packages (https://github.com/corot/turtlebot_arm, https://github.com/turtlebot/turtlebot_arm) are included.



All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

Remarks
-------

In visualizatoin the separation of both fingers of the gripper is just an linear approximation of the real euclidean distance. However, this is ok for our purposes
right now.

Note, this package includes an (slighly) adopted version of the turtlebot_arm package (resp. the branch https://github.com/corot/turtlebot_arm.git).
It is copied into this metapackage in order to allow a simple integration for students that start working with the PhantomX.
