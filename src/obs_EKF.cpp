#include "ros/ros.h"
#include "EKF/EKF.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>


class Node {
private:
    ros::NodeHandle nh_;
    ros::Subscriber marker_sub_,ego_pos_sub_;
    ros::Publisher obs_goal_pub_, min_obs_pre_pub_;
    ros::Timer timer_;
    std::unordered_map<int, EKF_CV*> ekf_map_;
    Eigen::Vector3d ego_pos_;
    int min_ID = INT16_MAX;
    double min_dist = INT16_MAX;

    inline double norm(Eigen::Vector2d vec) {
        return sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
    }
    inline bool isFrontCar(Eigen::Vector2d obs_pos,Eigen::Vector3d  ego_pos) {
        Eigen::Vector2d vec = obs_pos - ego_pos.head(2);
        double angle = std::atan2(vec(1), vec(0)) - ego_pos(2);
        return (angle >= -M_PI / 4 && angle <= M_PI / 4);
    }
    inline void risk_ellipse(Eigen::Matrix2d P, double risk_level, double& a, double& b, double& theta) {
        Eigen::EigenSolver<Eigen::Matrix2d> es(P);
        Eigen::VectorXcd eigvals = es.eigenvalues();
        Eigen::MatrixXcd eigvecs = es.eigenvectors();
        double c = -2*log(risk_level);
        a = sqrt(c * eigvals[0].real())+0.2;
        b = sqrt(c * eigvals[1].real())+0.2;
        theta = atan2(eigvecs(1,0).real(), eigvecs(0,0).real());
    }

public: 
    

    Node(ros::NodeHandle& nh): nh_(nh){  
        ego_pos_ << 0, 0, 0;   
        marker_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/mot_tracking/box",1,&Node::markerCallback,this);
        ego_pos_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom",1,&Node::egoPosCallback,this);
        obs_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/obs_goal",1);
        min_obs_pre_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/min_obs_pre",1);
        timer_ = nh_.createTimer(ros::Duration(0.1), &Node::timerCallback, this);
    }
    ~Node(){}; 

    void egoPosCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        ego_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z;
        std::cout << "Ego position: " << ego_pos_ << std::endl;
    }

    void timerCallback(const ros::TimerEvent& event) {

        if(ekf_map_.size() != 0 && min_ID != INT16_MAX){
            std::vector<Eigen::VectorXd> X_pred;
            std::vector<Eigen::MatrixXd> P_pred;
            ekf_map_[min_ID]->self_predict(10, X_pred,  P_pred);
            visualization_msgs::MarkerArray min_obs_pre_array;
            std::vector<std::vector<double>> AFb_array;
            int pre_id = 0;
            for(auto prei : X_pred){
                Eigen::Matrix2d mat2d = P_pred[pre_id].block<2, 2>(0, 0);
                double a,b,theta;
                risk_ellipse(mat2d,0.1,a,b,theta);
                tf2::Quaternion q;
                q.setRPY(0, 0, theta); // 绕Z轴旋转 theta 弧度

                visualization_msgs::Marker obs_pre;
                obs_pre.header.frame_id = "os_sensor";
                obs_pre.header.stamp = ros::Time::now();
                obs_pre.ns = "obs_pre";
                obs_pre.id = pre_id++;
                obs_pre.type = visualization_msgs::Marker::SPHERE;
                obs_pre.action = visualization_msgs::Marker::ADD;
                obs_pre.pose.position.x = prei[0];
                obs_pre.pose.position.y = prei[1];
                obs_pre.pose.position.z = 0;
                obs_pre.pose.orientation.x = q.x();
                obs_pre.pose.orientation.y = q.y();
                obs_pre.pose.orientation.z = q.z();
                obs_pre.pose.orientation.w = q.w();
                obs_pre.scale.x = a; 
                obs_pre.scale.y = b; 
                obs_pre.scale.z = 0; 
                obs_pre.color.r = 0.59f;
                obs_pre.color.g = 0.98f;
                obs_pre.color.b = 0.59f;
                obs_pre.color.a = 0.8 - 0.05*pre_id;
                // obs_pre.lifetime = ros::Duration();
                min_obs_pre_array.markers.push_back(obs_pre);
                AFb_array.push_back({prei[0],prei[1],a,b,theta});
            }
            min_obs_pre_pub_.publish(min_obs_pre_array);

            

        }
        else{
            ROS_WARN("There is no obs!");
        }
       
    }

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        
        for (const auto& marker : msg->markers) {
            int id = marker.id;
            Eigen::Vector2d z;
            z << marker.pose.position.x, marker.pose.position.y;

            if (ekf_map_.find(id) == ekf_map_.end()) {
                Eigen::VectorXd x0(6);
                x0 << z, 0, 0, 0, 0;
                Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6);
                Eigen::MatrixXd Q0 = Eigen::MatrixXd::Identity(6, 6) * 0.1;
                Eigen::MatrixXd R0 = Eigen::MatrixXd::Identity(2, 2) * 0.1;
                double dt0 = 0.1;

                ekf_map_.insert(std::pair<int, EKF_CV*>(id,new EKF_CV()));
                ekf_map_[id]->initialize(x0, P0, Q0, R0, dt0);
            }

            ekf_map_[id]->predict();
            ekf_map_[id]->update(z);

            Eigen::VectorXd state = ekf_map_[id]->getState();
            Eigen::Vector2d obs_pos = state.head(2);
            double dist = norm(obs_pos - ego_pos_.head(2));
            if(isFrontCar(obs_pos,ego_pos_)){
                
                if (dist < min_dist) {
                    min_dist = dist;
                    min_ID = id;
                }
            }
            ROS_INFO("Obstacle ID: %d, x: %f,y: %f", id, state[0], state[1]);
        }
        visualization_msgs::Marker obs_goal;
        obs_goal.header.frame_id = "os_sensor";
        obs_goal.header.stamp = ros::Time::now();
        obs_goal.ns = "obs_goal";
        obs_goal.id = 0;
        obs_goal.type = visualization_msgs::Marker::ARROW;
        obs_goal.action = visualization_msgs::Marker::ADD;
        obs_goal.pose.position.x = ekf_map_[min_ID]->getState()[0];
        obs_goal.pose.position.y = ekf_map_[min_ID]->getState()[1];
        obs_goal.pose.position.z = 2;
        obs_goal.pose.orientation.x = 1.0;
        obs_goal.pose.orientation.y = 0.0;
        obs_goal.pose.orientation.z = 0.0;
        obs_goal.pose.orientation.w = 0.0;
        // 设置箭头的大小
        obs_goal.scale.x = 1; // 箭头杆的长度
        obs_goal.scale.y = 0.5; // 箭头杆的直径
        obs_goal.scale.z = 0.3; // 箭头头部的直径
        obs_goal.color.r = 1.0f;
        obs_goal.color.g = 0.0f;
        obs_goal.color.b = 0.0f;
        obs_goal.color.a = 1.0;
        obs_goal.lifetime = ros::Duration();
        
        obs_goal_pub_.publish(obs_goal);
    }
       
};






int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "obs_EKF");
    ros::NodeHandle nh;
    Node Node_(nh);

    ros::spin();


    return 0;
}
