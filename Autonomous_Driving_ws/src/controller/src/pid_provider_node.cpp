#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pid/pid.h>
#include <ros/ros.h>

class PidProviderNode {
  ros::NodeHandle nh;

  ros::Subscriber target_twist_sub;
  ros::Subscriber current_state_sub;

  ros::Publisher target_v_pub;
  ros::Publisher current_v_pub;

  // ros::Publisher target_omega_pub;
  // ros::Publisher current_omega_pub;
public:
  PidProviderNode() {
    target_twist_sub = nh.subscribe("target_twist", 1,
                                    &PidProviderNode::extractPidTarget, this);
    current_state_sub = nh.subscribe(
        "current_state_est", 1, &PidProviderNode::extractCurrentTwist, this);

    target_v_pub = nh.advertise<std_msgs::Float64>("target_linear_velocity", 1);
    current_v_pub =
        nh.advertise<std_msgs::Float64>("current_linear_velocity", 1);

    // current_omega_pub =
    // nh.advertise<std_msgs::Float64>("current_angular_velocity", 1);
    // target_omega_pub =
    // nh.advertise<std_msgs::Float64>("target_angular_velocity", 1);
  }
  void extractPidTarget(const geometry_msgs::Twist twist) {
    std_msgs::Float64 v;
    v.data = twist.linear.x;
    target_v_pub.publish(v);

    // std_msgs::Float64 omega;
    // omega = twist.angular.z;
    // target_omega_pub.publish(omega);
  }

  void extractCurrentTwist(const nav_msgs::Odometry current_state_est) {
    std_msgs::Float64 v;
    v.data = current_state_est.twist.twist.linear.x;
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
