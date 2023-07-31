#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pid/pid.h>
#include <ros/ros.h>

class CurrentTwistNode {
  /**
   * @class CurrentTwistNode
   * @brief Node that publishes the current twist of the vehicle
   *
   */
  ros::NodeHandle nh;

  ros::Subscriber current_state_sub;

  ros::Publisher current_v_pub;
  ros::Publisher current_omega_pub;

public:
  CurrentTwistNode() {
    // Subcribers
    current_state_sub = nh.subscribe(
        "current_state_est", 1, &CurrentTwistNode::extractCurrentTwist, this);

    // Publishers
    current_v_pub =
        nh.advertise<std_msgs::Float64>("current_linear_velocity", 1);
    current_omega_pub =
        nh.advertise<std_msgs::Float64>("current_angular_velocity", 1);
  }

  void extractCurrentTwist(const nav_msgs::Odometry current_state_est) {
    /**
     * @brief Extracts the current twist from the current_state_est messagej
     *
     * @param current_state_est Current state estimation
     */
    std_msgs::Float64 v;
    v.data = current_state_est.twist.twist.linear.x;
    current_v_pub.publish(v);

    std_msgs::Float64 omega;
    omega.data = current_state_est.twist.twist.angular.z;
    current_omega_pub.publish(omega);
  }
};

int main(int argc, char *argv[]) {
  /**
   * @brief Main function
   *
   * @param argc
   * @param argv
   */
  // Initialize ROS
  ros::init(argc, argv, "current_twist");

  // Create object
  CurrentTwistNode pid_provider_node;

  // Spin
  ros::spin();

  return 0;
}
