#include "std_msgs/Bool.h"
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float64.h>

class controllerNode {
  ros::NodeHandle nh;

  ros::Subscriber twist_sub;
  ros::Subscriber pid_effort_sub;
  ros::Subscriber traffic_state_sub;

  ros::Publisher car_commands_pub;

  double linear_acc;
  double omega;
  bool stop_signal;

  ros::Timer timer;
  double hz; // frequency of the main control loop
public:
  controllerNode() : hz(1000) {
    twist_sub =
        nh.subscribe("target_twist", 1, &controllerNode::updateOmega, this);
    pid_effort_sub =
        nh.subscribe("linear_acc", 1, &controllerNode::updateLinearAcc, this);
    traffic_state_sub = nh.subscribe("traffic_light_state", 1,
                                     &controllerNode::updateStopSignal, this);

    car_commands_pub = nh.advertise<mav_msgs::Actuators>("car_commands", 1);

    timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

    stop_signal = false;
  }

  void updateOmega(geometry_msgs::Twist twist) { omega = twist.angular.z; }

  void updateLinearAcc(std_msgs::Float64 effort) { linear_acc = effort.data; }

  void updateStopSignal(std_msgs::Bool traffic_light_state) {
    stop_signal = traffic_light_state.data;
  }

  void controlLoop(const ros::TimerEvent &t) {
    mav_msgs::Actuators msg;

    if (stop_signal) {
      msg.angular_velocities.resize(4);
      msg.angular_velocities[0] = 0;  // Acceleration
      msg.angular_velocities[1] = 0;  // Turning angle
      msg.angular_velocities[2] = 10; // Breaking
      msg.angular_velocities[3] = 0;
    } else {
      msg.angular_velocities.resize(4);
      msg.angular_velocities[0] = linear_acc; // Acceleration
      msg.angular_velocities[1] = omega;      // Turning angle
      msg.angular_velocities[2] = 0;          // Breaking
      msg.angular_velocities[3] = 0;
    }

    car_commands_pub.publish(msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}
