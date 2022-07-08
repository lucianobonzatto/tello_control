#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

geometry_msgs::Pose telloPose;
int detect;
int tag;

void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
  if(msg->markers.size()>0){
    telloPose.position.x = msg->markers[0].pose.pose.position.x;
    telloPose.position.y = msg->markers[0].pose.pose.position.y;
    telloPose.position.z = msg->markers[0].pose.pose.position.z;
    detect = 1;
    tag = msg->markers[0].id;
  }
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "pid_control");
  ros::NodeHandle nh(""), nh_param("~");
  ros::Rate loop_rate(10);
  int teste;
  ros::Subscriber pose_sub;
  pose_sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1, &poseCallback);
  detect = 0;
  tag = -1;
  int cont;
  int j = 0;

  std::ofstream myfile;
  myfile.open ("example.csv", std::ios::out);
  while(ros::ok()){
    getchar();
    tag = -1;
    cont =0;
    detect = 0;
    ROS_INFO("--------%d-----------", j);
    myfile << "\n\n\nX;Y;Z\n";
    while(cont < 100 && ros::ok()){
      ros::spinOnce();
      if(detect == 1){
        detect = 0;
        cont++;
        myfile << telloPose.position.x << ";" << telloPose.position.y << ";" << telloPose.position.z << "\n";
      }
    }
    ROS_INFO("+++++++++++++++++++++++");
    loop_rate.sleep();
    j++;
    if(j == 5){
      break;
    }
  }      
  myfile.close();
}
