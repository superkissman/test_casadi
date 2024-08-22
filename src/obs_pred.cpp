#include "ros/ros.h"
#include "EKF/EKF.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
EKF_CV obs1,obs2,obs3;
std::vector<EKF_CV*> ptrs = {&obs1,&obs2,&obs3};
// std::vector<EKF_CV*> ptrs = &obs1;
bool flag = true;

class Node {
public: 
    ros::NodeHandle nh_;
    ros::Publisher obs_EKF_pub;
    ros::Subscriber obs_sub;
    double delta_t = 0.1;
    visualization_msgs::MarkerArray marker_array;
    

    Node(ros::NodeHandle& nh){
        nh_ = nh;
        obs_sub = nh.subscribe<visualization_msgs::MarkerArray>("/obstacle_markers",1,&Node::obs_sub_callback,this);
        obs_EKF_pub = nh.advertise<visualization_msgs::MarkerArray>("/obs_EKF",30);
    }
    ~Node(){}; 

    void obs_sub_callback(const visualization_msgs::MarkerArray::ConstPtr& Msg){
        std::vector<Eigen::VectorXd> x;
        for(int k=0;k<Msg->markers.size();k++){
            Eigen::VectorXd x_tmp(6);
            x_tmp << Msg->markers[k].pose.position.x,Msg->markers[k].pose.position.y,0,0,0,0;
            x.push_back(x_tmp);
        }
        if(flag){
            flag = false;
            int k = 0;
            for(auto ptr : ptrs){
                ptr->initialize(x[0],Eigen::MatrixXd::Identity(6,6),0.01*Eigen::MatrixXd::Identity(6,6),0.01*Eigen::MatrixXd::Identity(2,2),0.1);
                k++;
            }
        }else{
            int k = 0;
            for(auto ptr : ptrs){
                ptr->predict();
                ptr->update(Eigen::Vector2d (x[k][0],x[k][1]));
                k++;
            }
            for(auto ptr : ptrs){
                std::vector<Eigen::VectorXd> X_pred;
                std::vector<Eigen::MatrixXd> P_pred;
                ptr->self_predict(10, X_pred, P_pred);
                // for(auto Pi:P_pred){
                //     std::cout << Pi << std::endl;
                //     std::cout << "***" << std::endl;
                // }
                // std::cout << "----------------" << std::endl;
                int i = 0;
                marker_array.markers.clear();
                for(auto vec : X_pred){
                    
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "/odom";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "obstacles";
                    marker.id = i;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = vec(0);
                    marker.pose.position.y = vec(1);
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 2.0;
                    marker.color.a = 0.6;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker_array.markers.push_back(marker);
                    
                    i++;
                }
                obs_EKF_pub.publish(marker_array);
            }
        }
        


    }
       
};






int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "obs_pred");
    ros::NodeHandle nh;
    Node Node_(nh);

    ros::spin();


    return 0;
}
