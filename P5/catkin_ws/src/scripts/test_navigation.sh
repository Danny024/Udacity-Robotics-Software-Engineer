#!/bin/sh

#TO DO
#Create a script to launch turtle bot world and to deploy a turtlebot in the environment
# use amcl to loaclize the turtlebot
# observe the map in rviz.

#define path
catkin_ws="/home/ubuntu/P5/catkin_ws"

#navigate to workspace, source the workspace and open the world
xterm -e "cd ${catkin_ws} && export SVGA_VGPU10=0 && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 15

#open a new terminal navigate to catkin workspace and launch AMCL
xterm -e "cd ${catkin_ws} && source devel/setup.bash && cd src && cd map && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${catkin_ws}/src/map/map.yaml" &

sleep 15

#open a new terminal navigate to catkin workspace and launch AMCL
xterm -e "cd ${catkin_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch"
