#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>
#include <thread>
#include <vector>


class Node {
public: 
    ros::NodeHandle nh_;
    ros::Publisher marker_pub;
    ros::Timer timer_;
    visualization_msgs::MarkerArray marker_array;


    Node(ros::NodeHandle& nh): nh_(nh){
        timer_ = nh_.createTimer(ros::Duration(0.1), &Node::timerCallback, this);
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/mot_tracking/box",1);
    }
    ~Node(){}; 

    void timerCallback(const ros::TimerEvent& event){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 5;
        marker.pose.position.y = 5;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 0.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }



    
       
};






int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "marker");
    ros::NodeHandle nh;
    Node Node_(nh);

    ros::spin();


    return 0;
}
