#!/bin/sh


# Catkin_ws path definition.
catkin_ws="/home/ubuntu/P5/catkin_ws"

# navigate to the catkin_ws, reduce the GPU for gazebo, source the catkin_ws to run and launch turtlebot_world.launch
xterm -e "cd ${catkin_ws} && export SVGA_VGPU10=0 && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# navigate to the catkin_ws, source and launch amcl_demo.launch
xterm -e "cd ${catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${catkin_ws}/src/map/playground.yaml" &

sleep 5

# navigate to the catkin_ws, source and launch view_home_service_navigation.launch
xterm -e "cd ${catkin_ws} && source devel/setup.bash && roslaunch add_markers home_service_navigation.launch rviz_path:=${catkin_ws}/src/config/config.rviz" &

sleep 5

# navigate to the catkin_ws, source and launch add_markers add_markers
xterm -e "cd ${catkin_ws} && source devel/setup.bash && rosrun add_markers add_markers" &

# navigate to the catkin_ws, source and launch pick_objects pick_objects
xterm -e "cd ${catkin_ws} && source devel/setup.bash && rosrun pick_objects pick_objects" 
