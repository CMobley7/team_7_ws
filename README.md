# Bayesian Robotics Workspace

## Install Dependencies

    cd ~/bayesian_robotics_ws
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y
    catkin_make

## Build Arena

    rosrun xacro xacro.py src/mbzirc_gazebo/worlds/mbzirc_arena.world.xacro > src/mbzirc_gazebo/worlds/mbzirc_arena.world

## Run Simulator

    source devel/setup.bash
    roslaunch mbzirc_gazebo mbzirc.launch

## Start UAV Controller

### Joystick

    source devel/setup.bash
    roslaunch uav_control teleop_joystick

Or

    rosrun joy joy_node
    rosrun uav_control teleop_joystick.py

### Keyboard

    source devel/setup.bash
    rosrun uav_control teleop_keyboard.py
