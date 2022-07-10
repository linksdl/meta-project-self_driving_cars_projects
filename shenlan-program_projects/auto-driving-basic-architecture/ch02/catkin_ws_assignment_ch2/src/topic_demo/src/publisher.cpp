#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <ros/time.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("demo/topic1", 1);
    ros::Rate loop_rate(1); // hz

    int count = 0;

    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        double time = ros::Time::now().toSec();
        ss << "the timestamp: " << time;
        msg.data = ss.str();

        ROS_INFO("send a message: %s", msg.data.c_str());  
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
