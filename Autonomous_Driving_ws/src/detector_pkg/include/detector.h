#ifndef DETECTOR_H
#define DETECTOR_H

#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32MultiArray.h"
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <vector>

class Detector {
public:
  Detector(ros::NodeHandle &nh);

  void semeticCallback(const sensor_msgs::ImageConstPtr &sem_img);
  void RGBCallback(const sensor_msgs::ImageConstPtr &sem_img);

  void recognize(const std::vector<uint32_t> pixels_in);
  // void timerCallback(const ros::TimerEvent&);
  std::vector<uint32_t> pixel_yellow_;

private:
  ros::Publisher pub_yellow_pixels_;
  ros::Publisher pub_traffic_state_;
  ros::Timer timer;

  ros::Subscriber sub_sem_cam_;
  ros::Subscriber sub_RGB_cam_;
  ros::ServiceServer service;
  ros::NodeHandle &nh_;

  int hight_ = 240;
  int width_ = 320;
  
};

#endif
