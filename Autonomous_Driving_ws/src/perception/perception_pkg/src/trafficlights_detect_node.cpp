#include "detector.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <detector.h>
#include <sstream>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trafficlights_detector");

  ros::NodeHandle n;

  Detector detector(n);
  detector.localize();
  detector.recognize();

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}