# team_7_ws
This repo contains a set of ROS nodes and applications which simulates MBZIRC Challenge-1

## Install Dependencies

    cd ~/bayesian_robotics_ws
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y
    catkin_make
    
##  Install The Following Packages

    ### Truck Related ROS Packages
    sudo apt-get install ros-indigo-husky-*
    sudo apt-get install ros-indigo-fetch-*
    
    ### Kalman Filter Related Packages
    sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose python-pip
    
    ### Upgrade scipy for filterpy
    sudo  pip install scipy --upgrade

    ### Install the filterpy package
    sudo pip install filterpy
