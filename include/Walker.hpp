/*
 MIT License

 Copyright (c) 2018 Rohit

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
 */

/**
 *  @file    Walker.hpp
 *  @author  Rohitkrishna Nambiar (rohit517)
 *  @date    11/19/2018
 *  @version 1.0
 *
 *  @brief Turtlebot walker like roomba
 *
 *  @section DESCRIPTION
 *
 *  Walker class header file to implement turtlebot walker.
 *
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

// C++ header files
#include <iostream>

// ROS header files
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Walker {
 public:
  /**
   *  @brief Default constructor for Walker class
   */
  Walker();

  /**
   *  @brief Default destructor for Walker class
   */
  ~Walker();

  /**
   *   @brief Function to process laser scan topic
   *
   *   @param scanMsg is a pointer to laser scan message
   *
   *   @return void
   */
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scanMsg);

  /**
   *   @brief Function to navigate the turtlebot and
   *          publish velocity commands
   *
   *   @return void
   */
  void walk();

 private:
  ros::NodeHandle nh;

  // Laser scan topic subscriber
  ros::Subscriber laserSub;

  // Flag to indicate if path is clear
  bool pathClear;

  // Variable to store turtlebot velocity
  geometry_msgs::Twist velMsg;

  // Geometry message twist publisher
  ros::Publisher velPub;

  // Laser scan threshold
  float laserRangeThreshold;
};

#endif  // INCLUDE_WALKER_HPP_
