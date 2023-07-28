/*This is a server, catch the request from client(judge) and send the pose form client 
  to move_base/goal.*/


#include <ros/ros.h>
#include "planning/PlanGoal.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "geometry_msgs/PointStamped.h"

class Sender{
    ros::NodeHandle nh;
    ros::ServiceServer serv;
    ros::Publisher pub;

    public:
    Sender(){
        serv = nh.advertiseService("set_pose",&Sender::doRequ,this);
        // pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",1);
        pub = nh.advertise<geometry_msgs::PointStamped>("/clicked_point",1);

    }

    bool doRequ(planning::PlanGoal::Request &req,
                planning::PlanGoal::Response &resp){
            
            try{
                resp.posex = req.posex_from_file;
                resp.posey = req.posey_from_file;
                resp.quaternionz = req.quaternionz_from_file;
                resp.quaternionw = req.quaternionw_from_file;

                // posstm.goal.target_pose.pose.position.x = resp.posex;
                // posstm.goal.target_pose.pose.position.y = resp.posey;
                // posstm.goal.target_pose.pose.orientation.x= 0;
                // posstm.goal.target_pose.pose.orientation.y = 0;
                // posstm.goal.target_pose.pose.orientation.z = resp.quaternionz;
                // posstm.goal.target_pose.pose.orientation.w = resp.quaternionw;
                // posstm.goal.target_pose.header.frame_id = "world";

                way_point.point.x = resp.posex;
                way_point.point.y = resp.posey;
                way_point.point.z = 0.0;
                way_point.header.frame_id = "world";

                pub.publish(way_point);

            }catch(const std::exception e){
                std::cout<<"sender didn't work:"<<e.what()<<std::endl;
                return false;
            }
            return true;
    }
    private:
    // move_base_msgs::MoveBaseActionGoal posstm;
    geometry_msgs::PointStamped way_point;
    };

int main(int argc, char* argv[]){
    ros::init(argc,argv,"sender");
    Sender sen;
    ROS_INFO("sender worked.");
    ros::spin();
    return 0;
}