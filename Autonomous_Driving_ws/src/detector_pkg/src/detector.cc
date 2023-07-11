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
  const std::vector<uint8_t> &image_data = sem_img->data;
  std_msgs::UInt32MultiArray msg_pixel_yellow;
  for (size_t i = 1; i < image_data.size(); i = i + 3) {
    if (image_data[i] == 235) {
      sem_area_trafficlights.push_back(i);
      // ROS_INFO("data[%zu]: %u", i, image_data[i]);
    }
  }
  // msg_pixel_yellow.data = pixel_yellow;
  // pub_yellow_pixels_.publish(msg_pixel_yellow);
}

void Detector::RGBCallback(const sensor_msgs::ImageConstPtr &RGB_img) {
  const std::vector<uint8_t> &image_data = RGB_img->data;

  std_msgs::Bool msg_traffic_state;
  for (size_t i = 0; i < sem_area_trafficlights.size(); i++) {
    // ROS_INFO("[%u] [%u]
    // [%u]",image_data[sem_area_trafficlights[i-1]],image_data[sem_area_trafficlights[i]],image_data[sem_area_trafficlights[i+1]]);
    if (image_data[sem_area_trafficlights[i] - 1] > 200 &&
        image_data[sem_area_trafficlights[i]] < 80 &&
        image_data[sem_area_trafficlights[i + 1]] < 80) {
      // ROS_INFO("Red Light!");
      msg_traffic_state.data = false;
      pub_traffic_state_.publish(msg_traffic_state);
    } else if (image_data[sem_area_trafficlights[i] - 1] < 100 &&
               image_data[sem_area_trafficlights[i]] > 200 &&
               image_data[sem_area_trafficlights[i + 1]] < 100) {
      msg_traffic_state.data = true;
      pub_traffic_state_.publish(msg_traffic_state);
    }
    else{
      //do nothing
    }
  }
}

// void Detector::recognize(std::vector<uint32_t> pixels_in) {

//   for (size_t i = 0; i < pixels_in.size(); i++) {
//     uint32_t pixel_index = (pixels_in[i] / 3) % (240 * 320);
//     int row = pixel_index / 320;
//     int col = pixel_index % 320;
//     // if (row <= 140 && col >= 100 && col <= 220) {
//     ROS_INFO("traffic light [%u] [%u]", col, row);
//     // }
//   }
// }