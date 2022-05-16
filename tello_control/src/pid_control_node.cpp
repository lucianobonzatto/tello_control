#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

geometry_msgs::Point gotoPose;
geometry_msgs::Pose telloPose;
std_msgs::Bool running;

ros::Publisher pose_pub;
ros::Publisher cmd_vel_pub;
int detect;

void gotoCallback(const geometry_msgs::Pose::ConstPtr& msg){
  gotoPose.x = msg->position.x;
  gotoPose.y = msg->position.y;
  gotoPose.z = msg->position.z;
}
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
  geometry_msgs::Pose out_msg;
  out_msg.position.x = 0;
  out_msg.position.y = 0;
  out_msg.position.z = 0;
  out_msg.orientation.x = 0;
  out_msg.orientation.y = 0;
  out_msg.orientation.z = 0;

  if(!msg->markers.empty()){
    telloPose.position.x = msg->markers[0].pose.pose.position.x;
    telloPose.position.y = msg->markers[0].pose.pose.position.y;
    telloPose.position.z = msg->markers[0].pose.pose.position.z;

    out_msg.position.x = msg->markers[0].pose.pose.position.x;
    out_msg.position.y = msg->markers[0].pose.pose.position.y;
    out_msg.position.z = msg->markers[0].pose.pose.position.z;
    out_msg.orientation.x = msg->markers[0].pose.pose.orientation.x;
    out_msg.orientation.y = msg->markers[0].pose.pose.orientation.y;
    out_msg.orientation.z = msg->markers[0].pose.pose.orientation.z;
    detect = 1;
  }
  else{
    detect = 0;
  }
  pose_pub.publish(out_msg);
}
void startCallback(const std_msgs::Bool::ConstPtr& msg){
  //running->data = (int) msg->data;
  if(running.data != msg->data){
    if(msg->data == 0){
      geometry_msgs::Twist cmdVelMsg;
      cmdVelMsg.linear.x = 0;
      cmdVelMsg.linear.y = 0;
      cmdVelMsg.linear.z = 0;
      cmdVelMsg.angular.x = 0;
      cmdVelMsg.angular.y = 0;
      cmdVelMsg.angular.z = 0;

      cmd_vel_pub.publish(cmdVelMsg);
    }
    running.data = (int) msg->data;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pid_control");
  ros::NodeHandle nh(""), nh_param("~");
  ros::Rate loop_rate(10);

  ros::Publisher flag_pub;
  ros::Subscriber goto_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber start_sub;

  float deltaX, deltaY, deltaZ;
  float last_deltaX = 0, last_deltaY = 0, last_deltaZ = 0;
  float cstP = 0.01, cstI = 0.01, cstD = 0.01;
  float cstP_eixoZ = 0.01, cstI_eixoZ = 0.01, cstD_eixoZ = 0.01;
  float maxVel = 1;
  float dist_max = 0.1;

  double time, last_time;

  detect = 0;

  nh_param.getParam("constante_P", cstP);
  nh_param.getParam("constante_I", cstI);
  nh_param.getParam("constante_D", cstD);
  nh_param.getParam("constante_P_Z", cstP_eixoZ);
  nh_param.getParam("constante_I_Z", cstI_eixoZ);
  nh_param.getParam("constante_D_Z", cstD_eixoZ);
  nh_param.getParam("DIST_MAX", dist_max);

  nh_param.param<double>("velocidade_maxima", maxVel);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);
  pose_pub = nh.advertise<geometry_msgs::Pose>("/pid_control/pose", 1);
  flag_pub = nh.advertise<std_msgs::Bool>("/pid_control/flag", 1);

  goto_sub = nh.subscribe<geometry_msgs::Pose>("/pid_control/goto", 1, &gotoCallback);
  pose_sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1, &poseCallback);
  start_sub = nh.subscribe<std_msgs::Bool>("/pid_control/start", 1, &startCallback);

  geometry_msgs::Twist cmdVelMsg;
  std_msgs::Bool flag_cheguei;
  cmdVelMsg.linear.x = 0;
  cmdVelMsg.linear.y = 0;
  cmdVelMsg.linear.z = 0;
  cmdVelMsg.angular.x = 0;
  cmdVelMsg.angular.y = 0;
  cmdVelMsg.angular.z = 0;

  gotoPose.x = 0;
  gotoPose.y = 0;
  gotoPose.z = 2;

  running.data = 0;

  while(ros::ok()){
    ros::spinOnce();

    nh_param.getParam("constante_P", cstP);
    nh_param.getParam("constante_I", cstI);
    nh_param.getParam("constante_D", cstD);
    nh_param.getParam("constante_P_Z", cstP_eixoZ);
    nh_param.getParam("constante_I_Z", cstI_eixoZ);
    nh_param.getParam("constante_D_Z", cstD_eixoZ);

    if(running.data == 0){
      cmdVelMsg.linear.x = 0;
      cmdVelMsg.linear.y = 0;
      cmdVelMsg.linear.z = 0;
      cmdVelMsg.angular.x = 0;
      cmdVelMsg.angular.y = 0;
      cmdVelMsg.angular.z = 0;
      last_deltaX = 0;
      last_deltaY = 0;
      last_deltaZ = 0;
      flag_cheguei.data = 0;
      flag_pub.publish(flag_cheguei);
      continue;
    }

    deltaX = telloPose.position.x - gotoPose.x;
    deltaY = telloPose.position.y - gotoPose.y;
    deltaZ = telloPose.position.z - gotoPose.z;
    time = ros::Time::now().toSec();
    
    flag_cheguei.data = 0;
    if(detect == 1){
      cmdVelMsg.linear.x = (-1)*cstP*deltaX + cstI*(deltaX*(time-last_time)) + cstD*(deltaX-last_deltaX)/(time-last_time);
      cmdVelMsg.linear.y = (cstP*deltaY + cstI*(deltaY*(time-last_time)) + cstD*(deltaY-last_deltaY)/(time-last_time));
      cmdVelMsg.linear.z = (cstP_eixoZ*deltaZ + cstI_eixoZ*(deltaZ*(time-last_time)) + cstD_eixoZ*(deltaZ-last_deltaZ)/(time-last_time));
      flag_cheguei.data = 0;

      if (cmdVelMsg.linear.x > maxVel)
          cmdVelMsg.linear.x = maxVel;
      if (cmdVelMsg.linear.x < -maxVel)
          cmdVelMsg.linear.x = -maxVel;

      if (cmdVelMsg.linear.y > maxVel)
          cmdVelMsg.linear.y = maxVel;
      if (cmdVelMsg.linear.y < -maxVel)
          cmdVelMsg.linear.y = -maxVel;

      if (cmdVelMsg.linear.z > maxVel)
          cmdVelMsg.linear.z = maxVel;
      if (cmdVelMsg.linear.z < -maxVel)
          cmdVelMsg.linear.z = -maxVel;
      
      if(abs(gotoPose.x - telloPose.position.x) < dist_max){
        if(abs(gotoPose.y - telloPose.position.y) < dist_max){
          if(abs(gotoPose.z - telloPose.position.z) < dist_max){
            flag_cheguei.data = 1;
            cmdVelMsg.linear.x = 0;
            cmdVelMsg.linear.y = 0;
            cmdVelMsg.linear.z = 0;
          } 
        }
      }
    }
    else{
      cmdVelMsg.linear.x = 0;
      cmdVelMsg.linear.y = 0;
      cmdVelMsg.linear.z = 0;
    }
    
    // ROS_INFO("goto X: %f, goto Y: %f, goto Z: %f", gotoPose.x, gotoPose.y, gotoPose.z);
    // ROS_INFO("pose X: %f, pose Y: %f, pose Z: %f", telloPose.position.x, telloPose.position.y, telloPose.position.z);
    ROS_INFO("cmdv X: %f, cmdv Y: %f, cmdv Z: %f", cmdVelMsg.linear.x, cmdVelMsg.linear.y, cmdVelMsg.linear.z);
    // ROS_INFO("cstP: %f, cstI: %f, cstD: %f", cstP, cstI, cstD);
    // ROS_INFO("cstPz: %f, cstIz: %f, cstDz: %f", cstP_eixoZ, cstI_eixoZ, cstD_eixoZ);
    // ROS_INFO("cheguei: %i, running: %i", flag_cheguei.data, running.data);
    ROS_INFO("\n\n");

    if(running.data == 1){
      cmd_vel_pub.publish(cmdVelMsg);
      flag_pub.publish(flag_cheguei);
    }
    
    last_deltaX = deltaX;
    last_deltaY = deltaY;
    last_deltaZ = deltaZ;
    last_time = time;

    loop_rate.sleep();
  }
}
