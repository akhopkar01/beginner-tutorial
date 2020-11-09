/******************************************************************************************
 * Copyright: Aditya Khopkar | akhopkar@umd.edu
 *  
 * @file: talker.cpp
 * @brief: Publisher node to subscribe to a topic
 * @author: Aditya Khopkar
 * ****************************************************************************************/


#include <sstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials/ChangeString.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief: Struct wrapper for string message
 * @param: None
 * @return: None 
 */
struct NewMsg {
  std::string newString{"This is Week 9! enpm808x"};
} nm;


/**
 * @brief: Service callback function to change string
 * @param: Service request, Service response
 * @return: bool
 */
bool changeString(beginner_tutorials::ChangeString::Request& req,
                  beginner_tutorials::ChangeString::Response& res) {
  res.output = req.input;
  nm.newString = res.output;
  ROS_WARN_STREAM("The string is being changes due to service call");
  return true;
}

/**
 * @brief: main function to establish publisher
 * @param: Command argument count (argc), command argument (argv)
 * @return: Exit code 0
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  float freq{0};
  if (argc > 1) {
    freq = atoi(argv[1]);
  }
  ROS_DEBUG_STREAM("The frequency has been set to : " << freq);
  if (freq == 0) {
    ROS_ERROR_STREAM("Frequency cannot be < or = 0");
    freq = 1;
    ROS_WARN_STREAM("Setting frequency to lowest " << freq);
  }
  if (freq < 0) {
    ROS_FATAL_STREAM("Frequency cannot be negative. Terminating!!");
    system("rosnode kill /Talker");
    system("rosnode kill /Listener");
    ros::shutdown();
  }

  // Create publisher node to the topic 'chatter'
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer serve = n.advertiseService("ChangeString", changeString);
  ros::Rate loop_rate(freq);

  // Initialize count of message
  int count = 0;

  // Create broadcaster intance
  static tf::TransformBroadcaster br;

  // Create quaternion & transform instance
  tf::Transform transform;

  while (ros::ok()) {
    // Set origin for the transform frame
    transform.setOrigin(tf::Vector3(1.0, 2.0, 0.0));

    // Set quaternion -> (Roll, Pitch Yaw) using fixed axis
    tf::Quaternion q;
    q.setRPY(0, 0, 10);

    // Set rotation of the transform using quaternion
    transform.setRotation(q);

    // Send the transform using @param instance of transform, time stamp, parent frame, child frame
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    
    // Message object of topic std_msgs::String
    std_msgs::String msg;
    std::stringstream ss;
    ss << nm.newString << count;
    msg.data = ss.str();

    // Display ROS info
    ROS_INFO_STREAM("Message : " << msg.data.c_str());

    // publish function to publish the message
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

