#include "ros/ros.h"
#include "std_msgs/String.h" 


void msgCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("listener receive a message: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener"); 
    ros::NodeHandle n; 
    ros::Subscriber sub = n.subscribe("demo/topic1", 1, msgCallback);
    ros::spin();
    return 0;
}




