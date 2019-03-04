#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <math.h>

ros::Publisher uwbposPub;
ros::Publisher uwbvelPub;
geometry_msgs::PointStamped uwbpos;
geometry_msgs::TwistStamped uwbvel;

int main(int argc, char *argv[])  
{
     ros::init(argc, argv, "uwb_2");
     ros::NodeHandle nh;
     ROS_INFO("KJK");
     system("/home/xuanlingmu/startposition.sh");
    ros::spin();
	return 0;
}


