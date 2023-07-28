#include <ros/ros.h>
#include "planning/PlanGoal.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath> // for output pose in while-loop

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

void dealPose(const geometry_msgs::PoseStamped& curr_state){
    // ROS_INFO("what I get is %f.",curr_state.pose.position.x);
    x = curr_state.pose.position.x;
    y = curr_state.pose.position.y;
}

void updateStopSignal(const std_msgs::Bool& traffic_light_state) {
  // if (traffic_light_state.data != stop_signal) {
  //   counter += 1;
  // }
  // if (counter >= 5) {
  //   counter = 0;
    stop_signal = traffic_light_state.data;
    // ROS_INFO_STREAM(stop_signal);
  // }
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
    //1
    {5.66, -63.88},
    {5.06, -63.88},    /************** begin!*/
    {-27.0, -63.88},  /*   go straight      */
    // 2
    {-35.7, -62.16},  /* big turn right*/
    {-42.60, -56.7},
    {-47.1, -50.4},
    {-49.8, -43.96},
    {-50.6, -35.6},
    // 3
    {-52.73603057861328, -14.326130867004395},
    // 4
    {-52.261474609375,39.646183013916016},
    // 5
    { -52.709590911865234,121.1639404296875},
    //trafic light
    {-14.26545524597168,122.87753295898438},
    {-14.00545524597168,122.87753295898438},

    // 6
    { -5.211203575134277,123.36095428466797},
    //trafic light
    {-3.168438196182251,216.4018096923828},
    {-3.168438196182251,216.9018096923828},

    // 7
    { -3.231934070587158,227.98741149902344},
    // 8
    {-29.54465103149414,229.5660858154297},
    // 9
    {-37.97798156738281,227.83343505859375},
    {-44.626346588134766,220.97134399414062},
    {-48.98228073120117,210.46121215820312},
    {-50.98259353637695,199.9217071533203},
    // 5
    { -52.709590911865234,121.1639404296875},
    // 10
     {-52.04221954345703, 46.51329803466797},
    // traffic light
    {-12.776530265808105, 45.32085418701172},
    {-12.06530265808105, 45.32085418701172},


    // 11
    {-4.41776237487793, 44.37269592285156},
    // 12
    {-4.440919876098633, 20.33664321899414},
    // 13
    {-2.304708480834961, -2.790951728820801},
    //14 traffic light
    {-2.308427104949951,-49.423377990722656},
    {-2.308427104949951,-49.923377990722656},

    // 15
    {-2.4671823978424072,-62.84307098388672},
    // 16
    {4.748623847961426,-62.9009895324707},
    {4.948623847961426,-62.9009895324707},

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

  ros::Subscriber get_cu = nh.subscribe("true_pose",1,&dealPose);
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
    
  // stop_signal = false;
  while(ros::ok()){
    
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

    distance_to_current_path_end = sqrt(pow(x - gloPoints[path_end_indices[path_counter-1]][0],2) + pow(y - gloPoints[path_end_indices[path_counter-1]][1],2));

    if (path_counter != 0 && distance_to_current_path_end < 5 && stop_signal == false){
      
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


    // ROS_INFO("distance to current path end is %f, current pose is (%d,%d),in path:%d.",distance_to_current_path_end,static_cast<int>(x),static_cast<int>(y),path_counter);

    ros::spinOnce();
  }
  return 0;
}