# team_7_ws
This repo contains a set of ROS nodes and applications which simulates MBZIRC Challenge-1

## Install Dependencies

    cd ~/bayesian_robotics_ws
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y
    catkin_make
