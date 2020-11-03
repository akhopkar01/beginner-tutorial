# ROS Package Beginner Tutorials
[![License](https://img.shields.io/badge/License-MIT%20-lightgreen.svg)](https://github.com/akhopkar01/beginner-tutorial/blob/master/LICENSE)
## About
This Respository consists of a ROS package for beginners for establishing a publisher and a subscriber node. The publisher publishes a string message onto the topic "chatter". The subscriber node subscribes to the corresponding topic. Service calls are made to the publisher to change the string the publisher publishes.
### Author
Aditya Khopkar, akhopkar@umd.edu

### Tutorial Steps
If you want to create your own ROS package, you may follow the steps as follows:
1. Step 1: [Navigating ROS Wiki](http://wiki.ros.org/ROS/Tutorials/NavigatingTheWiki)
2. Step 2: [Navigating the Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
3. Step 3: [Creating Catkin Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
4. Step 4: [Building the Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
5. Step 5: [Understanding Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
6. Step 6: [Understanding Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
7. Step 7: [Writing Publisher Subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
8. Step 8: [Examining Publisher Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber) 
9. Step 9: [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
10. Step 10: [Understanding Services and Params](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
11. Step 11: [Using rqt_console roslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)

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

We will now launch our package using ```roslaunch``` and call ```rosservice``` to change the output string
To execute the procedure, we do the following on the terminal:
```
1. cd ~/catkin_ws
2. catkin_make
3. source devel/setup.bash
4. roslaunch beginner_tutorials week9.launch freq:=<frequency argument>
``` 

The launch file takes a floating point argument to set the frequency and can be given to the ```roslaunch``` when evoked. For example,
```roslaunch beginner_tutorials week9.launch freq:=5```. Default value of frequency is 0 and the publisher will automatically set the frequency to 1 in such a case as the frequency cannot be 0, else no message will be published.

To invoke service calls to the nodes, you may do the following (NOTE: roslaunch must stay active):
On a new terminal:
```
cd ~/catkin_ws
source devel/setup.bash
rosservice call /ChangeString "<your string input>"
```
Ex: ```rosservice call /ChangeString "ENPM808X is AWESOME!!"```

You may observe the log messages on the terminal as well as rqt_console. To invoke rqt-console, open a new terminal (NOTE: roslaunch must stay active or alternatively, you may start roscore on a new terminal):
```rosrun rqt_console rqt_console```
 
