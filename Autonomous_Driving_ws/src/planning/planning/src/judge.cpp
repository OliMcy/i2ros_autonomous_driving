#include <ros/ros.h>
#include "planning/PlanGoal.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath> // for output pose in while-loop
#include "perception_msgs/TrafficState.h"

double x = 0.0; // define global variant, for current pose
double y = 0.0;

bool stop_signal = false;
int counter = 0;

double distance(const std::vector<double>& p1, const std::vector<double>& p2) {
  if (p1.size() != p2.size()) {
      // Handle the case where points have different dimensions
      return -1.0;
  }

  double sum = 0.0;
  for (size_t i = 0; i < p1.size(); ++i) {
      double diff = p2[i] - p1[i];
      sum += diff * diff;
  }

  return sqrt(sum);
}

void dealPose(const nav_msgs::Odometry& cur_state){
    // ROS_INFO("what I get is %f.",curr_state.pose.position.x);
    x = cur_state.pose.pose.position.x;
    y = cur_state.pose.pose.position.y;

}

void updateStopSignal(const perception_msgs::TrafficState& traffic_light_state) {
    stop_signal = traffic_light_state.state;
}

std::vector<int> findPointIndicesWithinDistance(const std::vector<std::vector<double>>& points) {
  std::vector<int> result;

  if (points.size() < 2) {
    // Handle the case where there are not enough points to compare distances
    return result;
  }

  for (size_t i = 1; i < points.size(); ++i) {
    double dist = distance(points[i], points[i - 1]);
    if (dist < 1.0) {
        // Save the index of the point with distance less than 1 to its previous point
        result.push_back(static_cast<int>(i));
    }
  }

  return result;
}

int main(int argc, char* argv[]){
  //The matrix for global-points
  std::vector<std::vector<double>> gloPoints = 
  {
    // 1
    {-20.00, -60.40},
    // 2
    {-33.60, -57.70},
    {-41.10, -52.40},
    {-47.80, -41.96},
    {-50.00, -33.600},
    // 3
    {-52.80, -14.32},
    // 4
    {-52.46, 39.64},
    // 5 turn
    { -52.60, 117.50},
    { -48.20, 121.20},
    //trafic light
    {-10.26, 122.87},
    {-10.00, 122.87},

    // 6 turn
    {-7.21, 123.00},
    {-4.51, 127.00},
    //trafic light
    {-3.56, 222.60},
    {-3.56, 223.10},

    // 7 turn
    {-4.10, 224.50},
    {-7.73, 228.48},
    // 8
    {-29.54, 229.56},
    // 9
    {-37.97, 226.23},
    {-44.62, 220.97},
    {-48.98, 210.46},
    {-50.98, 199.92},
    // 5
    {-52.20, 121.16},
    // 10 turn
    {-52.04, 50.51},
    {-48.04, 46.51},
    // traffic light
    {-10.00, 45.32},
    {-9.56, 45.32},

    // 11 turn
    {-8.01, 44.37},
    {-4.41, 40.37},
    // 12
    {-3.84, 20.33},
    // 13
    {-3.00, -5.79},
    //14 traffic light
    {-2.30, -59.00},
    {-2.30, -59.50},

    // 15
    {-2.46, -63.00},
    // 16
    {4.74, -63.90},
    {4.94, -63.90},

  };

  std::vector<int> path_end_indices = findPointIndicesWithinDistance(gloPoints);
  std::cout << "Coordinates of the found path end:" << std::endl;
  for (int index : path_end_indices) {
    std::cout <<"index:"<< index << "  pose:"<< "(";
    for (size_t i = 0; i < gloPoints[index].size(); ++i) {
        std::cout <<gloPoints[index][i];
        if (i != gloPoints[index].size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << ")" << std::endl;
  }

  ros::init(argc,argv,"judge");
  ros::NodeHandle nh;

  ros::Subscriber get_cu = nh.subscribe("odom",1,&dealPose);
  ros::Subscriber traffic_state_sub = nh.subscribe("/perception/traffic_state", 1,&updateStopSignal);
  double tol; // define tolerance
  double distance_to_current_path_end;

  /*service communication for pose setting, here is the client.*/
  ros::ServiceClient cli = nh.serviceClient<planning::PlanGoal>("set_pose");
  ros::service::waitForService("set_pose");
  

  planning::PlanGoal goal;
  uint8_t point_counter = 0;
  uint8_t path_counter = 0;
  uint8_t current_path_begin = 0;
  uint8_t current_path_end = 0;
    
  while(ros::ok()){
    // publish the first path when the first traffic light is green
    if( path_counter==0 && stop_signal == false){
      distance_to_current_path_end = sqrt(pow(x - gloPoints[path_end_indices[0]][0],2) + pow(y - gloPoints[path_end_indices[0]][1],2));
      for(int i=0; i<= path_end_indices[0]; i++){
        goal.request.posex_from_file = gloPoints[point_counter][0];
        goal.request.posey_from_file = gloPoints[point_counter][1];
        bool flag = cli.call(goal);
        if(flag){
          ROS_INFO("pose %d in path %dsuccessfully set!",point_counter+1, path_counter);
          ROS_INFO("distance to current path end: %f:",distance_to_current_path_end);
        }else{
          ROS_INFO("pose setting failed...");
          return 1;
        }
        point_counter++;
      }
      path_counter++;
    }

    //calculate the distance to the current path end
    distance_to_current_path_end = sqrt(pow(x - gloPoints[path_end_indices[path_counter-1]][0],2) + pow(y - gloPoints[path_end_indices[path_counter-1]][1],2));

    //pubilish the nexxt path only when the distance of the car to the curren path end is less than 2 and the traffic light is green 
    if (path_counter != 0 && distance_to_current_path_end < 2 && stop_signal == false){
      if(path_counter >= path_end_indices.size()) break;
      for(int i=path_end_indices[path_counter-1]+1; i<= path_end_indices[path_counter]; i++){
        goal.request.posex_from_file = gloPoints[point_counter][0];
        goal.request.posey_from_file = gloPoints[point_counter][1];
        bool flag = cli.call(goal);
        if(flag){
          ROS_INFO("pose %d in path %d successfully set!",point_counter + 1, path_counter);
          ROS_INFO("distance to current path end: %f:",distance_to_current_path_end);
        }else{
          ROS_INFO("pose setting failed...");
          return 1;
        }
        point_counter++;
      }
      path_counter++;
    }
    ros::spinOnce();
  }
  return 0;
}