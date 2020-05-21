#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>

int main(int argc, char **argv){
    //Initialize ROS, set up node
    ros::init(argc, argv, "Creel_Transform");
    ros::NodeHandle nh;
    
    //Create publisher
    ros::Publisher talker=nh.advertise<geometry_msgs::Point>("peg_position", 1,"True");
    
    //Pass coordinate values into message and Publish
    geometry_msgs::Point msg;
    msg.x = 0.0;
    msg.y = 0.5;
    msg.z = 0.0;
    talker.publish(msg);
        
    ROS_INFO("Peg Position Published");
        
    sleep(1);  
}
