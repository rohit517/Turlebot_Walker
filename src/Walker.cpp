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
 *  @file    walker.cpp
 *  @author  Rohitkrishna Nambiar (rohit517)
 *  @date    11/19/2018
 *  @version 1.0
 *
 *  @brief Turtlebot walker like roomba
 *
 *  @section DESCRIPTION
 *
 *  Walker slass file to implement turtlebot walker.
 *
 */

#include "../include/Walker.hpp"

Walker::Walker() {
  // Initialize threshold
  laserRangeThreshold = 0.70;

  // Subscribe to laser scan topic
  laserSub = nh.subscribe < sensor_msgs::LaserScan
      > ("/scan", 10, &Walker::processLaserScan, this);

  ROS_INFO("New turtlebot walker created. Object distance threshold set to %f",
           laserRangeThreshold);
}

Walker::~Walker() {

}

void Walker::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {
  // Set the minimum distance to the first distance in scan message
  float minDistance = scanMsg->ranges[0];

  // Iterate through the scan message to get the closest distance value
  for (int i = 0; i < scanMsg->ranges.size(); i++) {
    float currentScanVal = scanMsg->ranges[i];
    if (currentScanVal < minDistance) {
      minDistance = currentScanVal;
    }
  }

  if (minDistance < laserRangeThreshold) {
    ROS_INFO("Terrain!");
  }

//  ROS_INFO("Closet point distance %f", minDistance);
}

void Walker::walk() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {



    ros::spinOnce();
    loop_rate.sleep();
  }
}
