#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32MultiArray.h"
// #include "detection_msgs/BoundingBoxes.h"
// #include "detection_msgs/BoundingBox.h"
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
  const std::vector<uint8_t> &image_data = sem_img->data;

  for (size_t i = 0; i < 120; i++) {
    for (size_t j = 1; j < 960; j = j + 3) {
      if (image_data[i * 960 + j] == 235 && j > 300 && j < 660) {
        area_trafficlights_.push_back(i * 960 + j);
      }
    }
  }
}

void Detector::RGBCallback(const sensor_msgs::ImageConstPtr &RGB_img) {
  const std::vector<uint8_t> &image_data = RGB_img->data;

  std_msgs::Bool msg_traffic_state;
  for (size_t i = 0; i < area_trafficlights_.size(); i++) {
    // ROS_INFO("[%u] [%u]
    // [%u]",image_data[area_trafficlights_[i-1]],image_data[area_trafficlights_[i]],image_data[area_trafficlights_[i+1]]);
    if (image_data[area_trafficlights_[i] - 1] > 200 &&
        image_data[area_trafficlights_[i]] < 80 &&
        image_data[area_trafficlights_[i + 1]] < 80) {
      // ROS_INFO("Red Light!");
      msg_traffic_state.data = false;
      pub_traffic_state_.publish(msg_traffic_state);
    } else if (image_data[area_trafficlights_[i] - 1] < 100 &&
               image_data[area_trafficlights_[i]] > 200 &&
               image_data[area_trafficlights_[i + 1]] < 100) {
      msg_traffic_state.data = true;
      pub_traffic_state_.publish(msg_traffic_state);
    } else {
      // do nothing
    }
  }
}
