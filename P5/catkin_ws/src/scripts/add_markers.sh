#!/bin/sh



# Define catkin_ws path
catkin_ws="/home/ubuntu/P5/catkin_ws"

# Navigate to the catkin_ws, reduce the Video graphics for gazebo,source and launch turtlebot_world
xterm -e "cd ${catkin_ws} && export SVGA_VGPU10=0 && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Navigate to the catkin_ws, source and launch amcl_demo.launch
xterm -e "cd ${catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${catkin_ws}/src/map/map.yaml" &

sleep 5

# Navigate to the catkin_ws, source and launch view_navigation.launch
xterm -e "cd ${catkin_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Navigate to the catkin_ws, source and launch add_markers add_markers
xterm -e "cd ${catkin_ws} && source devel/setup.bash && rosrun add_markers add_markers" 
