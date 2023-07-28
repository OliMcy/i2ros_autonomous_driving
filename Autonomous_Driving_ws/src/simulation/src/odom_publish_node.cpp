#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <fla_utils/param_utils.h>

#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>
#include <random>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

class OdomPublishNode {
 public:
	OdomPublishNode() {
		// Subscribers
		current_odom_sub = nh.subscribe("/current_state_est", 1, &OdomPublishNode::CurrentOdom, this);

		// Publishers
		transformed_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

		// ros::NodeHandle pnh("~");


		// tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
	
	}

 private:

 	// geometry_msgs::PoseStamped pose_corrupted_;
	

	void CurrentOdom(nav_msgs::Odometry const& curent_odom){
		nav_msgs::Odometry tranformed_odom;
		Eigen::Vector3d v;  
		Eigen::Vector3d v_odom; 
		Eigen::Vector3d omega;
		Eigen::Vector3d omega_odom;
		
		tranformed_odom.header.stamp = curent_odom.header.stamp;
		tranformed_odom.header.frame_id = "world";
		tranformed_odom.child_frame_id = "base_link";

		Eigen::Quaterniond q;
		Eigen::Matrix3d R;
		tf::quaternionMsgToEigen (curent_odom.pose.pose.orientation, q);
		R = q.toRotationMatrix();
		Eigen::Matrix3d mat;
		mat << 0.0, -1.0, 0.0,
		    1.0, 0.0, 0.0,
		    0.0, 0.0, 1.0;
		Eigen::Matrix3d rot_mat;
		rot_mat = mat*R;
		Eigen::Quaterniond rot_q(rot_mat);

		tranformed_odom.pose.pose.orientation.x = rot_q.x();
		tranformed_odom.pose.pose.orientation.y = rot_q.y();
		tranformed_odom.pose.pose.orientation.z = rot_q.z();
		tranformed_odom.pose.pose.orientation.w = rot_q.w();
		// ROS_INFO_STREAM(tranformed_odom.twist.twist.linear);

		v << curent_odom.twist.twist.linear.x,curent_odom.twist.twist.linear.y,curent_odom.twist.twist.linear.z;
		omega << curent_odom.twist.twist.angular.x,curent_odom.twist.twist.angular.y,curent_odom.twist.twist.angular.z;

		Eigen::Quaterniond q2;
		tf::quaternionMsgToEigen (tranformed_odom.pose.pose.orientation, q2);
		R = q2.toRotationMatrix();
		v_odom = R.transpose()*v;
		omega_odom = R.transpose()*omega;

		tranformed_odom.twist.twist.linear.x = v_odom[0];
		tranformed_odom.twist.twist.linear.y = v_odom[1];
		tranformed_odom.twist.twist.linear.z = v_odom[2];
		tranformed_odom.twist.twist.angular.x = omega_odom[0];
		tranformed_odom.twist.twist.angular.y = omega_odom[1];
		tranformed_odom.twist.twist.angular.z = -omega_odom[2];
        tranformed_odom.pose.pose.position = curent_odom.pose.pose.position;

		transformed_odom_pub.publish(tranformed_odom);

	}




	bool virgin = true;
	size_t pose_count = 0;

	ros::NodeHandle nh;
	ros::Subscriber current_odom_sub;

	ros::Publisher transformed_odom_pub;

	// std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	// tf2_ros::Buffer tf_buffer_;

};


int main(int argc, char* argv[]) {
	std::cout << "Initializing odom_publish_node node" << std::endl;

	ros::init(argc, argv, "OdomPublishNode");

	OdomPublishNode odom_publish_node;

	ros::spin();

}
