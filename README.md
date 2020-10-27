# ROS Package Beginner Tutorials
## About
This Respository consists of a ROS package for establishing a publisher and a subscriber node. The publisher publishes a string message onto the topic "chatter". The subscriber node subscribes to the corresponding topic.
### Author
Aditya Khopkar, akhopkar@umd.edu

## Dependencies
The project depends on the following dependencies
* ROS-Melodic - Robotic Operating System is a meta-operating system for robots. The software can be installed from [here](http://wiki.ros.org/melodic/Installation/Ubuntu) 
* catkin - Catkin is build tool which is developed using CMake and GNU Make. You may install catkin [here](http://wiki.ros.org/catkin#Installing_catkin)

## Instructions
You need a workspace for this package. A workspace is a directory containing all the projects/packages along with their build and dependency files. To run a ROS package, a catkin workspace is recommended. First, we setup the catkin workspace for our ROS package -
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Now that we have setup our catkin workspace, we can clone this repository to build the package and execute - 
```
cd ~/catkin_ws/src
git clone https://github.com/akhopkar01/beginner_tutorials.git
cd beginner_tutorials
mkdir include
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

To execute the rosnodes, we do the following:
1. Open a new terminal
2. Run ```roscore```
3. Open a new terminal
4. Run ```source devel/setup.bash```
5. Run ```rosrun beginner_tutorials talker```
6. Open a new terminal
7. Run ```source devel/setup.bash```
8. Run ```rosrun beginner_tutorials listener``` 
 
