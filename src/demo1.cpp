#include "ros/ros.h"
#include "EKF/EKF.h"
#include "RISK_MPC/RISK_MPC.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>


class Node {
public: 
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub;
    ros::Subscriber obs_EKF_sub, ego_pos_sub;
    ros::Timer solver_timer;
    double delta_t = 0.1;
    visualization_msgs::MarkerArray marker_array;
    std::vector<double> ego_pos;
    std::vector<obs_circle> obs_array;
    RISK_MPC* mpc = new RISK_MPC();
    

    Node(ros::NodeHandle& nh){
        nh_ = nh;       
        mpc->initialize();
        solver_timer = nh.createTimer(ros::Duration(0.1),&Node::solver_callback,this);
        obs_EKF_sub = nh.subscribe<visualization_msgs::MarkerArray>("/obs_EKF",1,&Node::obs_EKF_callback,this);
        ego_pos_sub = nh.subscribe<nav_msgs::Odometry>("/ego_pos",1,&Node::ego_pos_callback,this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd/vel",10);
        ego_pos.resize(3);
    }
    ~Node(){}; 

    void ego_pos_callback(const nav_msgs::Odometry::ConstPtr& Msg){
        ego_pos[0] = Msg->pose.pose.position.x;
        ego_pos[1] = Msg->pose.pose.position.y;
        ego_pos[2] = Msg->pose.pose.position.z;
    }

    void obs_EKF_callback(const visualization_msgs::MarkerArray::ConstPtr& Msg){
        obs_array.clear();
        for(auto marker : Msg->markers){
            obs_circle obs;
            obs.x = marker.pose.position.x;
            obs.y = marker.pose.position.y;
            obs.r = 1.0;
            obs_array.push_back(obs);
        }
    }

    void solver_callback(const ros::TimerEvent& event){
        std::vector<double> goal_state = {15,0,0};
        
        if(obs_array.size() != 0){
            cmd cmd_ = mpc->compute(ego_pos,goal_state,obs_array);
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = cmd_.v;
            cmd_msg.angular.z = cmd_.w;
            cmd_pub.publish(cmd_msg);
        }
            
    }
       
};






int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "demo1");
    ros::NodeHandle nh;
    Node Node_(nh);

    ros::spin();


    return 0;
}
