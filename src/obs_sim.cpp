#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "obs_sim");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 10);
    
    double delta_t = 0.1;
    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < 3; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 5.0 + 5.0*i;
        marker.pose.position.y = -3.0 * i;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 2.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    ros::Rate r(10);
    while (ros::ok()){
        for (int i = 1; i < 3; ++i) {
            marker_array.markers[i].pose.position.y += 0.2*delta_t*i;
            marker_array.markers[i].header.stamp = ros::Time::now();           
        }
        // ROS_INFO("Publishing obstacle markers");
        marker_pub.publish(marker_array);
        r.sleep();
    }

    return 0;
}
