#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define PI M_PI


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;


  ros::Subscriber current_state;
  ros::Subscriber command_vel;
  ros::Subscriber ackermann_cmd;
  ros::Publisher car_commands;
  ros::Timer timer;


  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  Eigen::Vector3d cmd_vel_linear_current;
  Eigen::Vector3d cmd_vel_angular;

  Eigen::Vector3d cmd_vel_linear_old;
  Eigen::Vector3d vel_ist;

  double ackermann_cmd_steering_angle;
  double ackermann_cmd_vel;


public:
  controllerNode():hz(100.0){
      
      current_state = nh.subscribe("odom", 1, &controllerNode::onCurrentState, this);
      command_vel = nh.subscribe("cmd_vel", 1, &controllerNode::onCurrentCmdVel, this);
      ackermann_cmd = nh.subscribe("ackermann_cmd", 1, &controllerNode::onAckermannCmd, this);
      car_commands = nh.advertise<mav_msgs::Actuators>("car_commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      
    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();
    vel_ist = v;
    // ROS_INFO_STREAM(vel_ist[0]<<vel_ist[1]<<vel_ist[2]);


    // Rotate omega
    omega = R.transpose()*omega;
    // ROS_INFO_STREAM(omega);
  }

  void onCurrentCmdVel(const geometry_msgs::Twist& cmd_velocity){
      cmd_vel_linear_current << cmd_velocity.linear.x,cmd_velocity.linear.y,cmd_velocity.linear.z;
      cmd_vel_angular << cmd_velocity.angular.x,cmd_velocity.angular.y,cmd_velocity.angular.z;

      cmd_vel_linear_old = cmd_vel_linear_current;
  }

  void onAckermannCmd(const ackermann_msgs::AckermannDriveStamped& ackermann_cmd){
      ackermann_cmd_steering_angle = ackermann_cmd.drive.steering_angle;
      ackermann_cmd_vel = ackermann_cmd.drive.speed;
  }


  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = -(vel_ist[0]-ackermann_cmd_vel)*5; // Acceleration
    msg.angular_velocities[1] = -ackermann_cmd_steering_angle*2;  // Turning angle
    msg.angular_velocities[2] = 0; // Breaking
    msg.angular_velocities[3] = 0;
    car_commands.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
