# ROS Package Beginner Tutorials
[![License](https://img.shields.io/badge/License-MIT%20-green.svg)](https://github.com/akhopkar01/beginner-tutorial/blob/master/LICENSE)
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
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

Now that we have setup our catkin workspace, we can clone this repository to build the package and execute - 
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/akhopkar01/beginner_tutorials.git
$ cd beginner_tutorials
$ mkdir include
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
### Usage - roslaunch and rosservice (Week9)

In this subsection, we will now launch our package using ```roslaunch``` and call ```rosservice``` to change the output string
To execute the procedure, we do the following on the terminal:
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch beginner_tutorials week9.launch freq:=<frequency argument>
``` 

The launch file takes a floating point argument to set the frequency and can be given to the ```roslaunch``` when evoked. For example,
```roslaunch beginner_tutorials week9.launch freq:=5```. Default value of frequency is 0 and the publisher will automatically set the frequency to 1 in such a case as the frequency cannot be 0, else no message will be published.

To invoke service calls to the nodes, you may do the following (NOTE: roslaunch must stay active):
On a new terminal:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /ChangeString "<your string input>"
```
Ex: ```rosservice call /ChangeString "ENPM808X is AWESOME!!"```

You may observe the log messages on the terminal as well as rqt_console. To invoke rqt-console, open a new terminal (NOTE: roslaunch must stay active or alternatively, you may start roscore on a new terminal):
```rosrun rqt_console rqt_console```
 
### Usage - tf, rostest, rosbag, UnitTest(gtest) (Week10)

Perhaps, in this sub-section we will be using ```rostest```, ```rosbag``` and Unit testing in our ROS package. The source code is modified and updated to take into account /tf package of ROS. 

#### tf

In this, we implement a transform broadcaster to broadcast a static transform between the talker node (/talk) frame and the /world frame. To understand tf, follow the following steps:

1. Step 1: [Introduction to tf](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf)
2. Step 2: [Writing a tf_broadcaster](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29)
3. Step 3: [Writing a tf_listener](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29)
4. Step 4: [Adding a frame](http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29)
5. Step 5: [tf and Time](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29)
6. Step 6: [Time travel with tf](http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29)

In order to inspect the tf frames, you may do as follows:
```
$ sudo apt-get install ros-melodic-ros-tutorials ros-melodic-geometry-tutorials ros-melodic-rviz ros-melodic-rosbash ros-melodic-rqt-tf-tree

$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch beginner_tutorials week9.launch freq:=1
```
Open a new terminal and do the following:
```
$ rosrun rqt_tf_tree rqt_tf_tree
```
OR, you may simply:
```
$ rqt &
```
And then select, Plugins->Visualization->TF_tree. You may view the tf frames by: 
```
$ rosrun tf view_frames
$ evince frames.pdf
```
You may also execute tf_echo functionality by running the command: ```rosrun tf tf_echo talk world```. Follow Step 1 for further clarification. 

The frames.pdf could also be accessed from beginner_tutorials/results/frames.pdf.

#### rostest
This package includes rostest suite which includes unit-testing framework using g-test. The rostest may be executed and validated as follows, Open a terminal:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rostest beginner_tutorials test.launch
```
There are 2 tests, the Summary should suggest RESULT:SUCCESS. You may follow the following links for details regarding rostest:
1. [rostest-Writing](http://wiki.ros.org/rostest/Writing)
2. [gtest ROS](http://wiki.ros.org/gtest)

#### rosbag
```rosbag``` is a package majorly used to record data and simulate in simulation time, which emulates real-time. The rosbag functionality can be enabled for the package as follows in your catkin workspace:
```
$ source devel/setup.bash
$ roslaunch beginner_tutorials week9.launch freq:=1 rosbag:=enable
```
This starts the rosbag. The rosbag data is logged in beginner_tutorials/results/record.bag

In order to play the bag, navigate to the location suggested, Open a new terminal, After starting ```roscore``` in a new terminal:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
In the previous terminal, play the bag file using the command: ```rosbag play record.bag```. You will see the listener node generating messages received from the bag file, thus emulating the real functionality of our ROS package.

You can inspect the rosbag by following techniques:
```rosbag info record.bag``` To get the info of the recorded bagfile. You can see the /Chatter topic in the info.
```rosbag check record.bag``` TO check if the bagfile is playable or not. You may see this [link](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data) for reference.

### NOTE
It can be observed that, everytime a new terminal is opened you have to write the command ```source devel/setup.bash```. It gets really annoying to do so. The following workaround could be used:
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ./bashrc
```
By doing this, you wont have to worry about sourcing the setup file every time for your workspace.