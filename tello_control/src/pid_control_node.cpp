#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

geometry_msgs::Point gotoPose;
geometry_msgs::Pose telloPose;

void gotoCallback(const geometry_msgs::Pose::ConstPtr& msg){
  gotoPose.x = msg->position.x;
  gotoPose.y = msg->position.y;
  gotoPose.z = msg->position.z;
}
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
  if(!msg->markers.empty()){
    telloPose.position.x = msg->markers[0].pose.pose.position.x;
    telloPose.position.y = msg->markers[0].pose.pose.position.y;
    telloPose.position.z = msg->markers[0].pose.pose.position.z;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pid_control");
  ros::NodeHandle nh(""), nh_param("~");
  ros::Rate loop_rate(10);

  ros::Publisher cmd_vel_pub;
  ros::Subscriber goto_sub;
  ros::Subscriber pose_sub;

  float deltaX, deltaY, deltaZ;
  float last_deltaX, last_deltaY, last_deltaZ;
  float cstP = 0.01, cstI = 0.01, cstD = 0.01;
  float cstP_eixoZ = 0.01, cstI_eixoZ = 0.01, cstD_eixoZ = 0.01;
  float maxVel = 1;

  double time, last_time;

  nh_param.getParam("constante_P", cstP);
  nh_param.getParam("constante_I", cstI);
  nh_param.getParam("constante_D", cstD);
  nh_param.getParam("constante_P_Z", cstP_eixoZ);
  nh_param.getParam("constante_I_Z", cstI_eixoZ);
  nh_param.getParam("constante_D_Z", cstD_eixoZ);

  nh_param.param<double>("velocidade_maxima", maxVel);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);
  goto_sub = nh.subscribe<geometry_msgs::Pose>("/pid_control/goto", 1, &gotoCallback);
  pose_sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1, &poseCallback);

  geometry_msgs::Twist cmdVelMsg;
  cmdVelMsg.linear.x = 0;
  cmdVelMsg.linear.y = 0;
  cmdVelMsg.linear.z = 0;
  cmdVelMsg.angular.x = 0;
  cmdVelMsg.angular.y = 0;
  cmdVelMsg.angular.z = 0;

  while(ros::ok()){
    ros::spinOnce();

    nh_param.getParam("constante_P", cstP);
    nh_param.getParam("constante_I", cstI);
    nh_param.getParam("constante_D", cstD);
    nh_param.getParam("constante_P_Z", cstP_eixoZ);
    nh_param.getParam("constante_I_Z", cstI_eixoZ);
    nh_param.getParam("constante_D_Z", cstD_eixoZ);

    deltaX = telloPose.position.x - gotoPose.x;
    deltaY = telloPose.position.y - gotoPose.y;
    deltaZ = telloPose.position.z - gotoPose.z;
    time = ros::Time::now().toSec();

    cmdVelMsg.linear.x = (-1)*cstP*deltaX + cstI*(deltaX*(time-last_time)) + cstD*(deltaX-last_deltaX)/(time-last_time);
    cmdVelMsg.linear.y = (cstP*deltaY + cstI*(deltaY*(time-last_time)) + cstD*(deltaY-last_deltaY)/(time-last_time));
    cmdVelMsg.linear.z = (cstP_eixoZ*deltaZ + cstI_eixoZ*(deltaZ*(time-last_time)) + cstD_eixoZ*(deltaZ-last_deltaZ)/(time-last_time));
    
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

    ROS_INFO("goto X: %f, goto Y: %f, goto Z: %f", gotoPose.x, gotoPose.y, gotoPose.z);
    ROS_INFO("pose X: %f, pose Y: %f, pose Z: %f", telloPose.position.x, telloPose.position.y, telloPose.position.z);
    ROS_INFO("cmdv X: %f, cmdv Y: %f, cmdv Z: %f", cmdVelMsg.linear.x, cmdVelMsg.linear.y, cmdVelMsg.linear.z);
    ROS_INFO("cstP: %f, cstI: %f, cstD: %f", cstP, cstI, cstD);
    ROS_INFO("cstPz: %f, cstIz: %f, cstDz: %f", cstP_eixoZ, cstI_eixoZ, cstD_eixoZ);
    ROS_INFO("\n\n");

    cmd_vel_pub.publish(cmdVelMsg);
    
    last_deltaX = deltaX;
    last_deltaY = deltaY;
    last_deltaZ = deltaZ;
    last_time = time;

    loop_rate.sleep();
  }
}
