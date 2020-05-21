#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "geometry_msgs/Point.h"

void callback(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("Peg_Position: \n\tx=%.2f \n\ty=%.2f \n\tz=%.2f", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    
    ros::Subscriber listener = nh.subscribe("peg_position", 1, callback);
    
    ros::spin();
}
