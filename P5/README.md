<h1>Home Service Robot Project</h1>


[![Service Robot](https://img.youtube.com/vi/dylTKX7KMa4/0.jpg)](https://www.youtube.com/watch?v=dylTKX7KMa4)

## Project Description  
This project comprises of the following exercises :
### Mapping  
You will create a `test_slam.sh` script file and launch it to manually test SLAM.  
A functional map of the environment should be created which would be used for localization and navigation tasks.  
### Localization and Navigation  
You will create a `test_navigation.sh` script file to launch it for manual navigation test.  
Your robot should be able to navigate in the environment after a 2D Nav Goal command is issued.  
You will create a `pick_objects.sh` file that will send multiple goals for the robot to reach.  
The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone."  
### Home Service Functions  
You will create a `add_marker.sh` file that will publish a marker to rviz.  
The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.
The student should write a home_service.sh file that will run all the nodes in this project.  
The student's home service robot should be simulated as follow:  
* Initially show the marker at the pickup zone.
* Hide the marker once your robot reach the pickup zone.
* Wait 5 seconds to simulate a pickup.
* Show the marker at the drop off zone once your robot reaches it.


## Prerequisites/Dependencies  & Installations
* Gazebo >= 7.0  
* ROS Kinetic  
* Install ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* Install ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* Install ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* Install ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


## How to Set up workspace  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line and execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. On the command line and execute  
```
cd P5/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
5. Build and run your code.  

## File Management System 
Directory Structure  
```
P5                                                        # Home Service Robot Project
├── catkin_ws                                             # Catkin workspace
    ├── src
        ├── add_markers                                   # add_markers package        
        │   ├── launch
        │   │   ├── home_service_navigation.launch   # launch file for home service robot demo
        │   ├── src
        │   │   ├── add_markers.cpp                       # source code for add_markers node
        │   │   ├── add_markers_demo.cpp                  # source code for add_markers_demo
        ├── pick_objects                                  # pick_objects package     
        │   ├── src
        │   │   ├── pick_objects.cpp                      # source code for pick_objects node
        │   │   ├── pick_objects_demo.cpp                 # source code for pick_objects_demo
        ├── config                                        # rvizConfig package        
        │   ├── config.rviz                               # rvizConfig file for home service robot demo  
        ├── scripts                                       # shell scripts files
        │   ├── add_marker.sh                             # shell script to model virtual objects  
        │   ├── home_service.sh                           # shell script to launch home service robot demo  
        │   ├── pick_objects.sh                           # shell script to send multiple goals  
        │   ├── test_navigation.sh                        # shell script to test localization and navigation
        │   ├── test_slam.sh                              # shell script to test SLAM
        ├── slam_gmapping                                 # gmapping_demo.launch file
        ├── turtlebot                                     # keyboard_teleop.launch file
        ├── turtlebot_interactions                        # view_navigation.launch file
        ├── turtlebot_simulator                           # turtlebot_world.launch file package        
        ├── CMakeLists.txt                                # compiler instructions

```
- [view_home_service_navigation.launch](/P5/catkin_ws/src/add_markers/launch/home_service_navigation.launch): Launch rviz using rviz configuration  
- [add_markers.cpp](/P5/catkin_ws/src/pick_objects/src/add_markers.cpp): C++ script, communicate with `pick_objects` node and control the rviz marker appearance to simulate object pick up and drop off   
- [pick_objects.cpp](/P5/catkin_ws/src/pick_objects/src/pick_objects.cpp): C++ script, communicate with `add_markers` node and command the robot to pick up the object  
- [config.rviz](/P5/catkin_ws/src/rvizConfig/config.rviz): rvizConfig file for home service robot demo which contained `markers` option  
- [add_marker.sh](/P5/catkin_ws/src/scripts/add_marker.sh): Shell script file to deploy a turtlebot inside your environment, model a virtual object with markers in `rviz`.  
- [home_service.sh](/P5/catkin_ws/src/scripts/home_service.sh): Shell script file to deploy a turtlebot inside your environment, simulate a full home service robot capable of navigating to pick up and deliver virtual objects.  
- [pick_objects.sh](/P5/catkin_ws/src/scripts/pick_objects.sh): Shell script file to deploy a turtlebot inside your environment, communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.  
- [test_navigation.sh](/p5/catkin_ws/src/scripts/test_navigation.sh): Shell script file to deploy a turtlebot inside your environment, pick two different goals and test your robot's ability to reach them and orient itself with respect to them.  
- [test_slam.sh](/P5/catkin_ws/src/scripts/test_slam.sh): Shell script file to deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in `rviz`  

- [CMakeLists.txt](/catkin_ws/src/CMakeLists.txt): File to link the C++ code to libraries.  

## Run the project  
* Clone this repository
```
git clone https://github.com/Danny024/Udacity-Robotics-Software-Engineer/tree/main/P5.git
```
* Navigate to the `src` folder and clone the necessary repositories  
```
### Necessary Packages
cd P5/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
* Open the repository, make and source  
```
cd /home/workspace/P5/catkin_ws/
catkin_make
source devel/setup.bash
```
* Launch the home service robot
```
./src/scripts/home_service.sh
```
.

## Hints 
1. Update and upgrade your environment before running the code.  
```
sudo apt-get update && sudo apt-get upgrade -y
```
2. If your system python version from miniconda is python3 while the ros packages and tf are python2. A hack is to just set the system python to python2 via symbol link. Run the following commands to resolve it  
```
ln -s /usr/bin/python2 /root/miniconda3/bin/python
```
3. How to setup your environment at start up.  
```
Add the following line into the /home/workspace/.student_bashrc
export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages  
pip install catkin_pkg  
pip install rospkg  
```
4. How create package with dependencies  
```
catkin_create_pkg pick_objects move_base_msgs actionlib roscpp  
catkin_create_pkg add_markers roscpp visualization_msgs  
```
5. How to visualize your marker in the rviz  
To see the marker(virtual objects) demo, in addition to running the `./add_marker.sh`, you will need to manually add a 'Marker' in rviz with the following steps:  
* Find your rviz window  
* In the left bottom panel, click "Add" button  
* In 'By display type' tab, navigate the tree to 'rviz' then 'Marker'  
* Click 'OK' button  
* By now you should be able to see the marker(virtual objects) appear, disappear then appear again  

