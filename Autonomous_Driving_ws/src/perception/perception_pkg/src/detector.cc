#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32MultiArray.h"
#include <cstdint>
#include <cstdlib>
#include <detector.h>
#include <sstream>
#include <vector>


Detector::Detector(ros::NodeHandle &nh) : nh_(nh) {
  pub_traffic_state_ =
      nh.advertise<std_msgs::Bool>("/perception/traffic_state", 0);
  pub_BoundingBoxImage_ =
      nh.advertise<sensor_msgs::Image>("/perception/boundingbox_image", 0);
  msg_traffic_state_.data = true;
}

void Detector::semeticCallback(const sensor_msgs::ImageConstPtr &sem_img) {
  /**
   * Description:
   * Extract the coordinates of the trafflic light in semantic camera.
   * 
   * Input:
   * Image topic.
   * 
   * Output:
   * The index of the second channel of the pixel in data. 
   */

  // Get the image data
  const std::vector<uint8_t> &image_data = sem_img->data;
  // Get the width and height of the image
  int image_width = sem_img->width;
  int image_height = sem_img->height;
  // Loop through the image data
  // The traffic light only shows up in the upper half of the image
  for (size_t i = 0; i < image_height / 2; i++) {
    // Loop through the rows
    // The number of elements in each row equals the image_width * channels
    for (size_t j = 301; j < 660; j = j + 3) {
      /*
       * Check if the pixel belongs to traffic light located in middle part of image(300 ~ 600)
       * The pixel value of yellow is [255, 235, 4]
       * The value of the second channel will be examed, that's why j start from 301 but not 300.
       */ 
      if (image_data[i * 960 + j] == 235) {
        // Push the pixel coordinates into the vector
        area_trafficlights_.push_back(i * 960 + j);
      }
    }
  }
}

void Detector::RGBCallback(const sensor_msgs::ImageConstPtr &RGB_img) {
  const std::vector<uint8_t> &image_data = RGB_img->data;

  // publish traffic stateimage_data
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

      area_trafficlights_.clear();
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

      area_trafficlights_.clear();
    } 
    else {
      // do nothing
    }
  }
}

void Detector::getBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &msg) {
  const std::vector<uint8_t> &image_data = msg->data;
  int image_width = msg->width;
  int image_height = msg->height;

  // Variables to store the bounding box coordinates
  min_x_ = image_width; // Initialize with maximum possible value
  min_y_ = image_height;
  max_x_ = 0; // Initialize with minimum possible value
  max_y_ = 0;

  // Iterate over the pixels and analyze RGB values
  for (int y = 0; y < 120; ++y) {
    for (int x = 140; x < 180; ++x) {
      int pixel_index =
          (y * image_width + x) * 3; // Index of the pixel in the 1D vector

      // Extract RGB values of the pixel
      uint8_t red = image_data[pixel_index];
      uint8_t green = image_data[pixel_index + 1];
      uint8_t blue = image_data[pixel_index + 2];

      // Check if the pixel belongs to the segmented object
      // Here, you can define your own criteria based on RGB values
      if (red == 255 && green == 235 && blue == 4) {
        // Update the bounding box coordinates if necessary
        if (x < min_x_)
          min_x_ = x;
        if (x > max_x_)
          max_x_ = x;
        if (y < min_y_)
          min_y_ = y;
        if (y > max_y_)
          max_y_ = y;
      }
    }
  }
}

void Detector::drawBoundingBoxCallback(
    const sensor_msgs::Image::ConstPtr &original_msg) {
  // Create a new image message for the modified image
  sensor_msgs::Image modified_msg;
  modified_msg.header = original_msg->header;
  modified_msg.height = original_msg->height;
  modified_msg.width = original_msg->width;
  modified_msg.encoding = original_msg->encoding;
  modified_msg.is_bigendian = original_msg->is_bigendian;
  modified_msg.step = original_msg->step;
  modified_msg.data = original_msg->data;

  // Iterate over the pixels within the bounding box region and modify the image
  // Modify the pixel color to highlight the bounding box
  // red
  if (msg_traffic_state_.data) {
    // Draw the top and bottom horizontal lines of the bounding box
    for (int x = min_x_; x <= max_x_; ++x) {
      // Top line
      int top_pixel_index = (min_y_ * modified_msg.width + x) * 3;
      modified_msg.data[top_pixel_index] = 0;       // Red channel
      modified_msg.data[top_pixel_index + 1] = 255; // Green channel
      modified_msg.data[top_pixel_index + 2] = 0;   // Blue channel

      // Bottom line
      int bottom_pixel_index = (max_y_ * modified_msg.width + x) * 3;
      modified_msg.data[bottom_pixel_index] = 0;       // Red channel
      modified_msg.data[bottom_pixel_index + 1] = 255; // Green channel
      modified_msg.data[bottom_pixel_index + 2] = 0;   // Blue channel
    }

    // Draw the left and right vertical lines of the bounding box
    for (int y = min_y_; y <= max_y_; ++y) {
      // Left line
      int left_pixel_index = (y * modified_msg.width + min_x_) * 3;
      modified_msg.data[left_pixel_index] = 0;       // Red channel
      modified_msg.data[left_pixel_index + 1] = 255; // Green channel
      modified_msg.data[left_pixel_index + 2] = 0;   // Blue channel

      // Right line
      int right_pixel_index = (y * modified_msg.width + max_x_) * 3;
      modified_msg.data[right_pixel_index] = 0;       // Red channel
      modified_msg.data[right_pixel_index + 1] = 255; // Green channel
      modified_msg.data[right_pixel_index + 2] = 0;   // Blue channel
    }
  } else {
    // Draw the top and bottom horizontal lines of the bounding box
    for (int x = min_x_; x <= max_x_; ++x) {
      // Top line
      int top_pixel_index = (min_y_ * modified_msg.width + x) * 3;
      modified_msg.data[top_pixel_index] = 255;   // Red channel
      modified_msg.data[top_pixel_index + 1] = 0; // Green channel
      modified_msg.data[top_pixel_index + 2] = 0; // Blue channel

      // Bottom line
      int bottom_pixel_index = (max_y_ * modified_msg.width + x) * 3;
      modified_msg.data[bottom_pixel_index] = 255;   // Red channel
      modified_msg.data[bottom_pixel_index + 1] = 0; // Green channel
      modified_msg.data[bottom_pixel_index + 2] = 0; // Blue channel
    }

    // Draw the left and right vertical lines of the bounding box
    for (int y = min_y_; y <= max_y_; ++y) {
      // Left line
      int left_pixel_index = (y * modified_msg.width + min_x_) * 3;
      modified_msg.data[left_pixel_index] = 255;   // Red channel
      modified_msg.data[left_pixel_index + 1] = 0; // Green channel
      modified_msg.data[left_pixel_index + 2] = 0; // Blue channel

      // Right line
      int right_pixel_index = (y * modified_msg.width + max_x_) * 3;
      modified_msg.data[right_pixel_index] = 255;   // Red channel
      modified_msg.data[right_pixel_index + 1] = 0; // Green channel
      modified_msg.data[right_pixel_index + 2] = 0; // Blue channel
    }
  }
  pub_BoundingBoxImage_.publish(modified_msg);
}

void Detector::localize() {
  sub_sem_cam_ =
      nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1000,
                    &Detector::semeticCallback, this);
}

void Detector::recognize() {
  sub_RGB_cam_ = nh_.subscribe("/realsense/rgb/left_image_raw", 1000,
                               &Detector::RGBCallback, this);
}

void Detector::getBoundingBox() {
  sub_getBoundingbox_ =
      nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1000,
                    &Detector::getBoundingBoxCallback, this);
}

void Detector::drawBoundingBox() {
  sub_drawBoundingBox_ =
      nh_.subscribe("/realsense/rgb/left_image_raw", 1000,
                    &Detector::drawBoundingBoxCallback, this);
}
