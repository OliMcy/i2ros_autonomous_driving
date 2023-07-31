/*This is a client, to judge if the condition is fullfilled to 
  send next pose point.*/


#include <ros/ros.h>
#include "planning/PlanGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath> // for output pose in while-loop

double x = 0.0; // define global variant, for current pose
double y = 0.0;

/*get current pose, to global x,y.*/
void dealPose(const geometry_msgs::PoseStamped& curr_state){
    x = curr_state.pose.position.x;
    y = curr_state.pose.position.y;
}


int main(int argc, char* argv[]){

// set strictly horizontal direction for the car. (used in the qudternion when drive towards left or right)
double along_y = std::sqrt(2.0)/2.0;


//The matrix for global-points
  
std::vector<std::vector<double>> gloPoints = {
{8.66, -62.88, 0.999798, 0.02009},
{-17.0, -62.0, 0.999798, 0.02009},
{-40.29, -58.0, 0.92417, 0.38196},
{-52.00, -33.44, along_y, along_y},
{-52.30, -14.56, along_y, along_y},
{-51.00, 10.00, along_y, along_y},    // *point1!!! along_y
{-51.00, 24.00, along_y, along_y},    // interpo along_y
{-51.30, 39.00, along_y, along_y},    // along_y
{-51.0, 53.81, along_y, along_y},    // along_y
{-51.00, 70.00, along_y, along_y},    // *point2!!! interpo along_y
{-51.00, 85.00, along_y, along_y},    // interpo along_y
{-51.0, 104.49, along_y, along_y},
{-51.0, 116.00, along_y, along_y},   // interpo along_y
{-38.45, 124.0, 0.00648, 0.99998},
{-25.0, 124.0, 0.00648, 0.99998},
{-10.00, 124.00, 0.01472, 0.99989},
{-3.0, 131.57, along_y, along_y},
{-4.5, 150.98, along_y, along_y},
{-4.0, 170.98, along_y, along_y},
{-4.0, 190.98, along_y, along_y},
{0.0, 205.70, along_y, along_y},
{0.0, 220.70, along_y, along_y},
{-13.0, 230.72, -0.99999, 0.00111},
{-25.33, 230.10, -0.99994, 0.10758},
{-45.94, 218.23, -0.80048, 0.59934},
{-51.43, 180.38, -along_y, along_y},
{-51.08, 154.32, -along_y, along_y},
{-52.27, 130.83, -along_y, along_y},
{-51.36, 106.52, -along_y, along_y},
{-53.77, 93.39, -along_y, along_y},
{-53.03, 70.57, -along_y, along_y},
{-53.41, 54.66, -along_y, along_y},
{-43.37, 45.89, 0.00895, 0.99996},
{-28.50, 46.19, -0.00951, 0.99995},
{-13.59, 45.43, 0.0018, 0.99999},
{-4.25, 36.59, -along_y, along_y},
{-4.25, 20.0, -along_y, along_y},
{-3.32, 7.04, -along_y, along_y},
{-2.57, -8.79, -along_y, along_y},
{-1.7, -38.1, -along_y, along_y},
{-0.18, -51.5, -along_y, along_y},
{-4.93, -61.21, -0.97428, 0.22534},
{-9.91, -62.84, 0.99999, 0.00074},
  };

  ros::init(argc,argv,"judge");
  ros::NodeHandle nh;

  ros::Subscriber get_cu = nh.subscribe("true_pose",1,&dealPose);
  double tol; // define tolerance

  //service communication for pose setting, here is the client.
  ros::ServiceClient cli = nh.serviceClient<planning::PlanGoal>("set_pose");
  ros::service::waitForService("set_pose");
  

  planning::PlanGoal goal;
  uint8_t counter = 0;

  //send the first point
  goal.request.posex_from_file = gloPoints[counter][0];
  goal.request.posey_from_file = gloPoints[counter][1];
  goal.request.quaternionz_from_file = gloPoints[counter][2];
  goal.request.quaternionz_from_file = gloPoints[counter][3];
    bool flag = cli.call(goal);
    if(flag){
        ROS_INFO("pose successfully set!");
    }else{
        ROS_INFO("pose setting failed...");
        return 1;
    }
  
  //set the rest points
  while(ros::ok()){
  ROS_INFO("tolerance is %f, current is (%d,%d),setted point:%d.",tol,static_cast<int>(x),static_cast<int>(y),counter);
     tol = abs(x - goal.request.posex_from_file) + abs(y - goal.request.posey_from_file);

      if (tol < 6){
        counter++;
        if(counter >= gloPoints.size()) break;
        goal.request.posex_from_file = gloPoints[counter][0];
        goal.request.posey_from_file = gloPoints[counter][1];
        bool flag = cli.call(goal);
        if(flag){
          ROS_INFO("pose %d successfully set!",counter + 1);
        }else{
          ROS_INFO("pose setting failed...");
          return 1;
      }
     }
    ros::spinOnce();
    }
    return 0;
}
