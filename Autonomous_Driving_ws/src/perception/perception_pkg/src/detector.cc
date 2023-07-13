#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32MultiArray.h"
#include <cstdint>
#include <detector.h>
#include <sstream>
#include <vector>

Detector::Detector(ros::NodeHandle &nh) : nh_(nh) {
  pub_traffic_state_ = nh.advertise<std_msgs::Bool>("traffic_state", 0);
  sub_sem_cam_ =
      nh.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1000,
                   &Detector::semeticCallback, this);
  ros::spinOnce();
  sub_RGB_cam_ = nh.subscribe("/realsense/rgb/left_image_raw", 1000,
                              &Detector::RGBCallback, this);
}

void Detector::semeticCallback(const sensor_msgs::ImageConstPtr &sem_img) {
  // Get the image data
  const std::vector<uint8_t> &image_data = sem_img->data;
  // Loop through the image data
  for (size_t i = 0; i < 120; i++) {
    // Loop through the rows
    for (size_t j = 1; j < 960; j = j + 3) {
      // Check if the pixel is a traffic light located in middle part of image
      if (image_data[i * 960 + j] == 235 && j > 300 && j < 660) {
        // Push the pixel coordinates into the vector
        area_trafficlights_.push_back(i * 960 + j);
      }
    }
  }
}

void Detector::RGBCallback(const sensor_msgs::ImageConstPtr &RGB_img) {
  const std::vector<uint8_t> &image_data = RGB_img->data;

  // publish traffic state
  for (size_t i = 0; i < area_trafficlights_.size(); i++) {
    // red light
    if (image_data[area_trafficlights_[i] - 1] > 200 &&
        image_data[area_trafficlights_[i]] < 100 &&
        image_data[area_trafficlights_[i + 1]] < 100) {
      ROS_INFO("Red Light!");
      // set traffic state to false
      msg_traffic_state_.data = false;
      // publish traffic state
      pub_traffic_state_.publish(msg_traffic_state_);
    }
    // green light
    else if (image_data[area_trafficlights_[i] - 1] < 150 &&
             image_data[area_trafficlights_[i]] > 200 &&
             image_data[area_trafficlights_[i + 1]] < 150) {
      ROS_INFO("Green Light!");
      // set traffic state to true
      msg_traffic_state_.data = true;
      // publish traffic state
      pub_traffic_state_.publish(msg_traffic_state_);
    }
     else {
      // do nothing
    }
  }
}