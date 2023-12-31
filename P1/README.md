<h1>The Build My World Project</h1>

![screenshot](https://github.com/Danny024/Udacity-Robotics-Software-Engineer/blob/main/images/myworld.png)  

## Project Description 
In this project you'll create your simulation world in Gazebo for all your upcoming projects in the [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209).  
1. Build a single floor wall structure using the **Building Editor** tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.  
2. Model any object of your choice using the **Model Editor** tool in Gazebo. Your model links should be connected with joints.  
3. Import your structure and two instances of your model inside an empty **Gazebo World**.  
4. Import at least one model from the **Gazebo online library** and implement it in your existing Gazebo world.  
5. Write a C++ **World Plugin** to interact with your world. Your code should display “Welcome to {YOUR_NAME}’s World!” message as soon as you launch the Gazebo world file.  
## Prerequisites/Dependencies  & Installations
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
## How to Set Up   
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  
## File Management System 
Directory Structure  
```
.P1                    # Build My World Project 
├── model                          # Model files 
│   ├── bookshelf
│   │   ├── model.config
│   │   ├── model.sdf
│   ├── coke
│   │   ├── model.config
│   |   ├── model.sdf
|   ├── cup
|   |   ├── model.config
|   |   ├── model.sdf
|   ├── floorplan
|   |   ├── model.config
|   |   ├── model.sdf
|   ├── robo
|   |    ├── model.config
|   |    ├── model.sdf
|   ├── table
|        ├── model.config
|        ├── model.sdf
├── script                         # Gazebo World plugin C++ script      
│   ├── welcome.cpp
├── world                          # Gazebo main World containing models 
│   ├── office.world
├── CMakeLists.txt                 # Link libraries 

```
## File links
- [office.world](/P1/world/office.world): Gazebo world file.  
- [floor](/P1/model/floorplan): Floor structure built by Building Editor of Gazebo.  
- [robot](/P1/model/robo): A robot built by Model Editor of Gazebo.  
- [welcome.cpp](/P1/script/welcome.cpp): Gazebo world plugin C++ script.  
- [CMakeLists.txt](CMakeLists.txt): File to link the C++ code to libraries.  
## How to Use 
* Clone this repository
* At the top level of the project repository, create a build directory:  
```bash
mkdir build && cd build
```
* In `/build` directory, compile your code with  
```bash
cmake .. && make
```
* Export your plugin folder in the terminal so your world file can find it:  
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/P1/build
```
* Launch the world file in Gazebo to load both the world and plugin  
```bash
cd /home/workspace/P1/world/
gazebo office.world
```

## Hints  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```

## Code Style

Adhere to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Marking Guide/Rubic 
### 1. Basic Requirements  
#### 1.1 Does the project include a world directory containing the Gazebo world file, a model directory containing a structure and an object model files, a script directory containing the C++ plugin code, and a CMakeLists.txt file?  
### 2. Building  
#### 2.1 Does the project include a house with walls?  
### 3. Modeling  
#### 3.1 Does the project include an object built using the Model Editor?   
### 4. Gazebo World  
#### 4.1 Does the project contain a Gazebo world with multiple models?  . 
### 5. World Plugin  
#### 5.1 Does the project contain a C++ world plugin?   
