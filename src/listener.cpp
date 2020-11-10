/******************************************************************************************
 * MIT License

Copyright (c) 2020 Aditya Khopkar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 * @file: listener.cpp
 * @brief: Subscriber node to subscribe to a topic
 * @author: Aditya Khopkar
 * ****************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief: Callback function for ROS node
 * @param: ConstPtr reference to message object of type std_msgs::String
 * @return: None
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard: " << msg->data.c_str());
}

/**
 * @brief: Main function  
 * @param: Command argument count (arg), command argument (argv)
 * @return: Exit code 0 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // Create subsciber node
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
