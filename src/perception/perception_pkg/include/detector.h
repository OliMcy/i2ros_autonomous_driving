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
  void RGBCallback(const sensor_msgs::ImageConstPtr &RGB_img);
  void getBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &msg);
  void drawBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &original_msg);
  void localize();
  void recognize();
  void getBoundingBox();
  void drawBoundingBox();

  // void timerCallback(const ros::TimerEvent&);

private:
  ros::Publisher pub_traffic_state_;
  ros::Publisher pub_BoundingBoxImage_;
  ros::Timer timer;

  ros::Subscriber sub_sem_cam_;
  ros::Subscriber sub_RGB_cam_;
  ros::Subscriber sub_getBoundingbox_;
  ros::Subscriber sub_drawBoundingBox_;
  ros::ServiceServer service;
  ros::NodeHandle &nh_;

  std_msgs::Bool msg_traffic_state_;
  sensor_msgs::Image msg_RGB_cam_;
  std::vector<uint32_t> area_trafficlights_;
  
  int min_x_, max_x_, min_y_, max_y_;
};

#endif
