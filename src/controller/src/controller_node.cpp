#include "std_msgs/Bool.h"
#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float64.h>

#define PI M_PI

class ControllerNode {
  /**
   * @class ControllerNode
   * @brief The controller node for the car
   *
   */
  ros::NodeHandle nh;

  ros::Subscriber linear_pid_effort_sub;
  ros::Subscriber omega_pid_effort_sub;
  ros::Subscriber traffic_state_sub;
  ros::Subscriber breaking_sub;
  ros::Subscriber current_v_sub;

  ros::Publisher car_commands_pub;

  double linear_acc;
  double turning_angle;

  ros::Timer command_timer;

  double hz; // frequency of the main control loop
public:
  ControllerNode() : hz(1000) {
    /**
     * @brief initialize the controller node
     */

    // Subscribers
    linear_pid_effort_sub =
        nh.subscribe("linear_acc", 1, &ControllerNode::updateLinearAcc, this);
    omega_pid_effort_sub = nh.subscribe(
        "turning_angle", 1, &ControllerNode::updateTurningAngle, this);
    current_v_sub = nh.subscribe("current_linear_velocity", 1,
                                 &ControllerNode::resetTurningAngle, this);

    // Publishers
    car_commands_pub = nh.advertise<mav_msgs::Actuators>("car_commands", 1);

    // Timers
    command_timer =
        nh.createTimer(ros::Rate(hz), &ControllerNode::controlLoop, this);

    // Initialize variables
    turning_angle = 0;
  }

  void resetTurningAngle(std_msgs::Float64 cur_v) {
    /**
     * @brief reset the turning angle to 0 when the car is moving backward
     *
     * @param cur_v current linear velocity of the car
     */
    if (cur_v.data < 0)
      turning_angle = 0;
  }

  void updateLinearAcc(std_msgs::Float64 effort) {
    /**
     * @brief update the linear acceleration of the car
     *
     * @param effort the effort from the linear pid controller
     */
    linear_acc = 1.5 * effort.data;
  }
  void updateTurningAngle(std_msgs::Float64 effort) {
    /**
     * @brief update the turning angle of the car
     *
     * @param effort the effort from the omega pid controller
     */
    turning_angle -= 0.1 * (effort.data);
    if (turning_angle < -1.2)
      turning_angle = -1.2;
    else if (turning_angle > 1.2)
      turning_angle = 1.2;
  }

  void controlLoop(const ros::TimerEvent &t) {
    /**
     * @brief main control loop
     *
     * @param t ros timer event
     */
    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = linear_acc;    // Acceleration
    msg.angular_velocities[1] = turning_angle; // Turning angle
    msg.angular_velocities[2] = 0;             // Breaking
    msg.angular_velocities[3] = 0;

    car_commands_pub.publish(msg);
  }
};

int main(int argc, char **argv) {
  /**
   * @brief main function
   *
   * @param argc
   * @param argv
   */
  // Initialize ROS
  ros::init(argc, argv, "controller_node");

  // Create our controller object and run it
  ControllerNode n;

  // Let ROS handle all callbacks
  ros::spin();

  return 0;
}
