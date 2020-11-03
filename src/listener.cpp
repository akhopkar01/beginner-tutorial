/******************************************************************************************
 * Copyright: Aditya Khopkar | akhopkar@umd.edu
 *  
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
