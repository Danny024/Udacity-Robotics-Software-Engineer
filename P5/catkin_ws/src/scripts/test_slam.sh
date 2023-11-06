#!/bin/sh

#To Do
# Deploy turtlebot in simulation environment
# Perform SLAM using gmapping
# Observe rviz map using the view_navigation
# Control the robot manually with kweyboard.

#Define catkin workspace
path="~/P5/catkin_ws"

#Navigate to the catkin_ws, source the workspace and launch the turtlebot world
xterm -e "export SVGA_VGPU10=0 && cd ${path} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 10

#Open a new terminal, navigate to workpsace source and launch gmapping_demo.launch
xterm -e "cd ${path} && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 10

#open a new terminal, source and launch the view_navigation
xterm -e "cd ${path} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 10

#Open a new terminal, source and launch keyboard_teleop
xterm -e "cd ${path} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
