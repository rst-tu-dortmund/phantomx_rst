# phantomx_rst
Things you need:

    sudo apt-get install ros-indigo-moveit-full
    cd path_to_your_catkin_ws/src
    git clone https://github.com/corot/turtlebot_arm.git    # This package contains the robot URDF model and the MoveIt configuration files
    git clone https://github.com/croesmann/arbotix_ros.git  # This package contains the low-level hardware driver (but supports a simulation mode)
    git clone https://github.com/rst-tu-dortmund/phantomx_rst.git # Here are a couple of launch files for our purposes and a robot API that can be used instead of Moveit
    
