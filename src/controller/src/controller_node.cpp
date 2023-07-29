#include "std_msgs/Bool.h"
#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float64.h>

#define PI M_PI

class controllerNode {
  ros::NodeHandle nh;

  // ros::Subscriber twist_sub;
  ros::Subscriber linear_pid_effort_sub;
  ros::Subscriber omega_pid_effort_sub;
  ros::Subscriber traffic_state_sub;
  ros::Subscriber breaking_sub;
  ros::Subscriber current_v_sub;

  ros::Publisher car_commands_pub;

  double linear_acc;
  double turning_angle;
  // double omega;

  ros::Timer command_timer;
  ros::Timer breaking_timer;
  double hz; // frequency of the main control loop
public:
  controllerNode() : hz(1000) {
    // twist_sub =
    //     nh.subscribe("target_twist", 1, &controllerNode::updateOmega, this);
    linear_pid_effort_sub =
        nh.subscribe("linear_acc", 1, &controllerNode::updateLinearAcc, this);
    omega_pid_effort_sub = nh.subscribe(
        "turning_angle", 1, &controllerNode::updateTurningAngle, this);
    current_v_sub = nh.subscribe("current_linear_velocity",1,&controllerNode::resetTurningAngle, this);
    car_commands_pub = nh.advertise<mav_msgs::Actuators>("car_commands", 1);

    command_timer =
        nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

    turning_angle = 0;
  }

  // void updateOmega(geometry_msgs::Twist twist) { omega = twist.angular.z; }

  void resetTurningAngle(std_msgs::Float64 cur_v)
  {
    if(cur_v.data < 0)
      turning_angle = 0;
  }

  void updateLinearAcc(std_msgs::Float64 effort) { linear_acc = 1.5 * effort.data; }
  void updateTurningAngle(std_msgs::Float64 effort) {
    turning_angle -= 0.1 * (effort.data);
    if( turning_angle < -1.2)
        turning_angle = -1.2;
    else if(turning_angle > 1.2)
        turning_angle = 1.2;
  }

  void controlLoop(const ros::TimerEvent &t) {
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
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}
