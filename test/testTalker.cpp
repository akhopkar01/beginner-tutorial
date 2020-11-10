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

 * @file: testTalker.cpp
 * @brief: test suites for Transform broadcaster and Service calls
 * @author: Aditya Khopkar
 * ****************************************************************************************/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <beginner_tutorials/ChangeString.h>
#include <std_msgs/String.h>

/**
 * @brief: This test tests Tranform Broadcaster. Checks if transform messages are broadcasted by 
 * employing a transform listener
 * @param: Name of the test case, name of the test 
 * */
TEST(TransformBroadcaster, tfListener) {
    tf::TransformListener listener;
    EXPECT_TRUE(listener.waitForTransform("world", "talk", ros::Time(),
                                          ros::Duration(2.0)));
}

/**
 * @brief: Tests the service call by calling a service client
 * @param: Name of the test case, name of the test
 * */
TEST(ChangeString, callService) {
    ros::NodeHandle n;
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::ChangeString>("ChangeString");
    EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));
}
