/*This is a client, to judge if the condition is fullfilled to 
  send next pose point.*/


// #include <ros/ros.h>
// #include "planning/PlanGoal.h"
// #include "geometry_msgs/PoseStamped.h"
// #include <vector>
// #include <string>
// #include <thread>

// /*This class is to get current pose of the car.*/
// class GetCurrentPose{


//   double x;
//   double y;
  

//   public:

//   GetCurrentPose(ros::NodeHandle nh){
//     ros::Subscriber curr_listen = nh.subscribe("true_pose",10, &GetCurrentPose::dealPose, this);
//     std::thread spinThread(&GetCurrentPose::spin, this);
//     spinThread.detach(); // 确保线程运行在后台
//   }

//   void dealPose(const geometry_msgs::PoseStamped& curr_state){
//     ROS_INFO("what I get is %f.",curr_state.pose.position.x);
//     x = curr_state.pose.position.x;
//     y = curr_state.pose.position.y;
//   }

//   /*get function, then we can get x out of calss.*/
//   double getPoseX(){
//     return x;
//   }

//   /*get function, then we can get y out of calss.*/
//   double getPoseY(){
//    return y;
//   }
//   /*spin function*/
//   void spin(){
//     ros::spin();
//   }
// };
// int main(int argc, char* argv[]){
//   //The matrix for global-points
//   std::vector<std::vector<double>> gloPoints = {
//     {8.66, -62.88, 0.99983, 0.01811},    /************** begin!*/
//     {-17.0, -62.88, 0.999798, 0.02009},  /*   go straight      */
//     {-40.29, -55.87, 0.92417, 0.38196},  /*   at the corner    */
//     {-51.105, -33.44, 0.71528, 0.69883}, /**** turn right over */
//     {-53.74, -14.56, 0.69673, 0.71733},
//     {-52,38, 56.81, 0.70711, 0.70710},
//     {-52.30, 106.49, 0.70232, 0.71215},
//     {-38.45, 122.13, 0.00648, 0.99998},
//     {-14.40, 123.40, 0.01472, 0.99989},
//     {-4.61, 131.57, 0.6962, 0.71785},
//     {-4.57, 167.98, 0.72802, 0.68556},
//     {-2.86, 209.70, 0.70344, 0.71075},
//     {-13.64, 229.72, -0.99999, 0.00111},
//     {-30.33, 229.10, -0.99994, 0.10758},
//     {-44.94, 218.23, -0.80048, 0.59934},
//     {-54.43, 180.38, -0.71915, 0.69485},
//     {-51.08, 154.32, -0.69158, 0.72230},
//     {-52.27, 130.83, -0.70711, 0.70711},
//     {-51.36, 106.52, -0.72175, 0.69215},
//     {-52.77, 93.39, -0.70385, 0.71034},
//     {-52.03, 70.57, -0.70446, 0.70974},
//     {-52.41, 54.66, -0.69384, 0.72013},
//     {-43.37, 45.89, 0.00895, 0.99996},
//     {-28.50, 46.19, -0.00951, 0.99995},
//     {-13.59, 45.43, 0.0018, 0.99999},
//     {-3.57, 36.59, -0.70515, 0.70905},
//     {-4.25, 20.0, -0.68091, 0.73236},
//     {-3.32, 7.04, -0.69608, 0.71797},
//     {-2.57, -8.79, -0.65667, 0.75417},
//     {-1.7, -38.1, -0.69401, 0.71996},
//     {-0.18, -51.5, -0.68278, 0.73062},
//     {-4.93, -61.21, -0.97428, 0.22534},
//     {-9.91, -62.84, 0.99999, 0.00074},
//   };

//     ros::init(argc,argv,"judge");
//     ros::NodeHandle nh;

//     GetCurrentPose getc(nh);
//     double tol; // tolerance

//     /*service communication for pose setting, here is the client.*/
//     ros::ServiceClient cli = nh.serviceClient<planning::PlanGoal>("set_pose");
//     ros::service::waitForService("set_pose");

//     planning::PlanGoal goal;
//     uint8_t counter = 0;

//     //send the first point
//     goal.request.posex_from_file = gloPoints[counter][0];
//     goal.request.posey_from_file = gloPoints[counter][1];
//      bool flag = cli.call(goal);
//       if(flag){
//           ROS_INFO("pose successfully set!");
//       }else{
//           ROS_INFO("pose setting failed...");
//           return 1;
//       }
    
//     while(ros::ok()){
//   // ROS_INFO("tolerance is %f, current x is %f.",tol,getc.getPoseX());
//      tol = abs(getc.getPoseX() - goal.request.posex_from_file) + abs(getc.getPoseY() - goal.request.posey_from_file);
//       if (tol < 1.0){
//         counter++;
//         if(counter >= gloPoints.size()) break;
//         goal.request.posex_from_file = gloPoints[counter][0];
//         goal.request.posey_from_file = gloPoints[counter][1];
//         bool flag = cli.call(goal);
//         if(flag){
//           ROS_INFO("pose successfully set!");
//         }else{
//           ROS_INFO("pose setting failed...");
//           return 1;
//       }
//      }
//     ros::spinOnce();
//     }
//     return 0;
// }


#include <ros/ros.h>
#include "planning/PlanGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath> // for output pose in while-loop

double x = 0.0; // define global variant, for current pose
double y = 0.0;

void dealPose(const geometry_msgs::PoseStamped& curr_state){
    // ROS_INFO("what I get is %f.",curr_state.pose.position.x);
    x = curr_state.pose.position.x;
    y = curr_state.pose.position.y;
}
int main(int argc, char* argv[]){
  //The matrix for global-points
  double along_y = std::sqrt(2.0)/2.0;
  std::vector<std::vector<double>> gloPoints = {
{8.66, -62.88, 0.999798, 0.02009},
{-17.0, -60.0, 0.999798, 0.02009},
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
{-14.00, 124.00, 0.01472, 0.99989},
{-4.5, 131.57, along_y, along_y},
{-4.5, 150.98, along_y, along_y},
{-4.0, 170.98, along_y, along_y},
{-4.0, 190.98, along_y, along_y},
{-2.86, 209.70, along_y, along_y},
{-13.64, 229.72, -0.99999, 0.00111},
{-30.33, 229.10, -0.99994, 0.10758},
{-44.94, 218.23, -0.80048, 0.59934},
{-54.43, 180.38, -along_y, along_y},
{-51.08, 154.32, -along_y, along_y},
{-52.27, 130.83, -along_y, along_y},
{-51.36, 106.52, -along_y, along_y},
{-52.77, 93.39, -along_y, along_y},
{-52.03, 70.57, -along_y, along_y},
{-52.41, 54.66, -along_y, along_y},
{-43.37, 45.89, 0.00895, 0.99996},
{-28.50, 46.19, -0.00951, 0.99995},
{-13.59, 45.43, 0.0018, 0.99999},
{-3.57, 36.59, -along_y, along_y},
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

    /*service communication for pose setting, here is the client.*/
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
    
  while(ros::ok()){
  ROS_INFO("tolerance is %f, current is (%d,%d),setted point:%d.",tol,static_cast<int>(x),static_cast<int>(y),counter);
     tol = abs(x - goal.request.posex_from_file) + abs(y - goal.request.posey_from_file);
      if (tol < 8){
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