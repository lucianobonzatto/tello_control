
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"


void gotoCallback(const geometry_msgs::Pose::ConstPtr& marker){

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pid_control");

  ros::NodeHandle nh(""), nh_param("~");
  ros::Publisher cmd_vel_pub;
  ros::Subscriber goto_sub;

  //nh_param->param<char>("cmd_vel_topic", enable_button, 5);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);
  goto_sub = nh.subscribe<geometry_msgs::Pose>("/pid_controll/goto", 1, &gotoCallback);

  ros::spin();
}
