#!/bin/sh


# Catkin_ws path definition
catkin_ws="/home/ubuntu/P5/catkin_ws"

# Navigate to the catkin_ws workspace, reduce graphic processing to avoid gazebo from dying, source and launch turtlebot_world
xterm -e "cd ${catkin_ws} && export SVGA_VGPU10=0 && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Navigate to the catkin_ws workspace, source and launch amcl_demo.launch
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${catkin_ws}/src/map/map.yaml" &

sleep 5

# Navigate to the catkin_ws workspace, source and launch view_navigation.launch
xterm -e "cd ${catkin_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Navigate to the w catkin_ws, source and launch pick_objects pick_objects_demo
xterm -e "cd ${catkin_ws} && source devel/setup.bash && rosrun pick_objects pick_objects" 
