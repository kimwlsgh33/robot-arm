# build sources
cd ~/catkin_ws && catkin_make && cd -

# launch ros
roslaunch open_manipulator_controller open_manipulator_controller.launch

# run teleop node
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
