#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <eigen3/Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>

class odometryPublishNode {
  ros::NodeHandle nh;

  ros::Subscriber current_state_sub;
  ros::Publisher odometry_pub;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener;
  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d v; // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R; // current orientation of the UAV
  Eigen::Vector3d
      omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  Eigen::Isometry3d eigen_tf_body_base;

public:
  odometryPublishNode() {

    current_state_sub = nh.subscribe(
        "current_state_est", 1, &odometryPublishNode::onCurrentState, this);
    odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry/raw", 1);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
  }

  void onCurrentState(const nav_msgs::Odometry &cur_state) {

    v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y,
        cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y,
        cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    geometry_msgs::TransformStamped tf_body_base;

    tf::quaternionMsgToEigen(cur_state.pose.pose.orientation, q);
    try {
      tf_body_base =
          tfBuffer.lookupTransform("body", "base_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      tf_body_base =
          tfBuffer.lookupTransform("body", "base_link", ros::Time(0));
    }

    tf::transformMsgToEigen(tf_body_base.transform, eigen_tf_body_base);
    R = q.toRotationMatrix();

    // Rotate omega
    omega = eigen_tf_body_base.rotation().transpose() * R.transpose() * omega;
    v = eigen_tf_body_base.rotation().transpose() * R.transpose() * v;
    nav_msgs::Odometry odometry;

    odometry.header.stamp = cur_state.header.stamp;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "base_link";

    odometry.pose = cur_state.pose;
    odometry.twist.twist.linear.x = v.x();
    odometry.twist.twist.linear.y = v.y();
    odometry.twist.twist.linear.z = v.z();
    odometry.twist.twist.angular.x = omega.x();
    odometry.twist.twist.angular.y = omega.y();
    odometry.twist.twist.angular.z = omega.z();
    odometry.twist.covariance = cur_state.twist.covariance;

    odometry_pub.publish(odometry);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publish_node");
  odometryPublishNode n;
  ros::spin();
}
