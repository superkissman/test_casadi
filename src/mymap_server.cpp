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
    ros::Publisher my_map_pub;
    ros::Subscriber marker_sub_, origin_map_sub;
    visualization_msgs::MarkerArray marker_array;
    nav_msgs::OccupancyGrid map;
    std::mutex buff_mutex_;
    

    Node(ros::NodeHandle& nh): nh_(nh){
        origin_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/map",1,&Node::origin_map_sub_callback,this);    
        marker_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/mot_tracking/box",1,&Node::markerCallback,this);    
        my_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/my_map",1);
    }
    ~Node(){}; 

    void origin_map_sub_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        buff_mutex_.lock();
        map = *msg;
        buff_mutex_.unlock();
    }

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
        buff_mutex_.lock();
        for(auto marker:msg->markers){
            double radius = marker.scale.x/2;
            double x = marker.pose.position.x;
            double y = marker.pose.position.y;
            int index_x = static_cast<int>((x - map.info.origin.position.x)/map.info.resolution);
            int index_y = static_cast<int>((y - map.info.origin.position.y)/map.info.resolution);
            int radius_len = static_cast<int>(radius/map.info.resolution);
            for(int i = -radius_len; i <= radius_len; i++){
                for(int j = -radius_len; j <= radius_len; j++){
                    if(index_x+i >= 0 && index_x+i < map.info.width && index_y+j >= 0 && index_y+j < map.info.height && (i*i+j*j) <= radius_len*radius_len){
                        map.data[index_x+i + (index_y+j)*map.info.width] = 100;
                    }
                }
            }
        }
        my_map_pub.publish(map);
        buff_mutex_.unlock();
    }

    
       
};






int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "mymap_server");
    ros::NodeHandle nh;
    Node Node_(nh);

    ros::spin();


    return 0;
}
