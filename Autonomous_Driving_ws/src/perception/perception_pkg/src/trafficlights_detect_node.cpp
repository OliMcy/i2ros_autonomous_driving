#include "detector.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <detector.h>
#include <sstream>
#include <vector>

int main(int argc, char **argv) {
  // %Tag(INIT)%
  ros::init(argc, argv, "trafficlights_detector");
  // %EndTag(INIT)%

  // %Tag(NODEHANDLE)%
  ros::NodeHandle n;
  // %EndTag(NODEHANDLE)%

  Detector detector(n);

  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%