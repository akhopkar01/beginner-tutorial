/******************************************************************************************
 * Copyright: Aditya Khopkar | akhopkar@umd.edu
 *  
 * @file: main.cpp
 * @brief: Primary getway to test suite API
 * @author: Aditya Khopkar
 * ****************************************************************************************/

#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief: main function to run all the tests
 * @param: command line agrument count, command line argument
 * @return: tests
 * */
int main(int argc, char** argv) {
  ros::init(argc, argv, "talkerTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
