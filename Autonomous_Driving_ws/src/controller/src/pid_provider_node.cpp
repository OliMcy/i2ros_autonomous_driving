#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pid/pid.h>
#include <ros/ros.h>

class PidProviderNode {
  ros::NodeHandle nh;

  ros::Subscriber target_twist_sub;
  ros::Subscriber current_state_sub;
  ros::Subscriber traffic_state_sub;

  ros::Publisher target_v_pub;
  ros::Publisher current_v_pub;

  // ros::Publisher target_omega_pub;
  // ros::Publisher current_omega_pub;

  bool stop_signal;
  int counter;

public:
  PidProviderNode() {
    target_twist_sub = nh.subscribe("target_twist", 1,
                                    &PidProviderNode::extractPidTarget, this);
    current_state_sub = nh.subscribe(
        "current_state_est", 1, &PidProviderNode::extractCurrentTwist, this);
    traffic_state_sub = nh.subscribe("perception/traffic_state", 1,
                                     &PidProviderNode::updateStopSignal, this);

    target_v_pub = nh.advertise<std_msgs::Float64>("target_linear_velocity", 1);
    current_v_pub =
        nh.advertise<std_msgs::Float64>("current_linear_velocity", 1);

    // current_omega_pub =
    // nh.advertise<std_msgs::Float64>("current_angular_velocity", 1);
    // target_omega_pub =
    // nh.advertise<std_msgs::Float64>("target_angular_velocity", 1);
    stop_signal = false;
  }
  void updateStopSignal(std_msgs::Bool traffic_light_state) {
    if (traffic_light_state.data != stop_signal) {
      counter += 1;
    }
    if (counter >= 5) {
      counter = 0;
      stop_signal = traffic_light_state.data;
    }
  }
  void extractPidTarget(const geometry_msgs::Twist twist) {
    std_msgs::Float64 v;

    if (stop_signal)
      v.data = 0.0;
    else
      v.data = twist.linear.x;

    target_v_pub.publish(v);

    // std_msgs::Float64 omega;
    // omega = twist.angular.z;
    // target_omega_pub.publish(omega);
  }

  void extractCurrentTwist(const nav_msgs::Odometry current_state_est) {
    std_msgs::Float64 v;
    v.data = -current_state_est.twist.twist.linear.x;
    current_v_pub.publish(v);

    // std_msgs::Float64 omega;
    // omega = twist.angular.z;
    // current_omega_pub.publish(omega);
  }
};

int main(int argc, char *argv[]) {
  std::cout << "Initializing PidProvder node" << std::endl;

  ros::init(argc, argv, "PidProviderNode");

  PidProviderNode pid_provider_node;

  ros::spin();
}
