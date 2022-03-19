#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Point gotoPose;
geometry_msgs::Pose telloPose;

void gotoCallback(const geometry_msgs::Pose::ConstPtr& msg){
  gotoPose.x = msg->position.x;
  gotoPose.y = msg->position.y;
  gotoPose.z = msg->position.z;
}
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
  telloPose.position.x = msg->position.x;
  telloPose.position.y = msg->position.y;
  telloPose.position.y = msg->position.z;
  telloPose.orientation.x = msg->orientation.x;
  telloPose.orientation.y = msg->orientation.y;
  telloPose.orientation.z = msg->orientation.z;
  telloPose.orientation.w = msg->orientation.w;

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
  float maxVel = 1;

  double time, last_time;

  nh_param.param<double>("constante_P", cstP);
  nh_param.param<double>("constante_I", cstI);
  nh_param.param<double>("constante_D", cstD);

  nh_param.param<double>("velocidade_maxima", maxVel);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);
  goto_sub = nh.subscribe<geometry_msgs::Pose>("/pid_control/goto", 1, &gotoCallback);
  pose_sub = nh.subscribe<geometry_msgs::Pose>("/tello/odom/marker", 1, &poseCallback);

  geometry_msgs::Twist cmdVelMsg;
  cmdVelMsg.linear.x = 0;
  cmdVelMsg.linear.y = 0;
  cmdVelMsg.linear.z = 0;
  cmdVelMsg.angular.x = 0;
  cmdVelMsg.angular.y = 0;
  cmdVelMsg.angular.z = 0;

  while(ros::ok()){
    ros::spinOnce();
    ROS_INFO("X: %f, Y: %f, Z: %f", gotoPose.x, gotoPose.y, gotoPose.z);

    deltaX = telloPose.position.x - gotoPose.x;
    deltaY = telloPose.position.y - gotoPose.y;
    deltaZ = telloPose.position.z - gotoPose.z;
    time = ros::Time::now().toSec();

    cmdVelMsg.linear.x = cstP*deltaX + cstI*(deltaX*(time-last_time)) + cstD*(deltaX-last_deltaX)/(time-last_time);
    cmdVelMsg.linear.y = cstP*deltaY + cstI*(deltaY*(time-last_time)) + cstD*(deltaY-last_deltaY)/(time-last_time);
    cmdVelMsg.linear.z = cstP*deltaZ + cstI*(deltaZ*(time-last_time)) + cstD*(deltaZ-last_deltaZ)/(time-last_time);
    
    if (cmdVelMsg.linear.x > maxVel)
        cmdVelMsg.linear.x = maxVel;

    if (cmdVelMsg.linear.y > maxVel)
        cmdVelMsg.linear.y = maxVel;

    if (cmdVelMsg.linear.z > maxVel)
        cmdVelMsg.linear.z = maxVel;
    
    cmd_vel_pub.publish(cmdVelMsg);
    
    last_deltaX = deltaX;
    last_deltaY = deltaY;
    last_deltaZ = deltaZ;
    last_time = time;

    loop_rate.sleep();
  }
}
