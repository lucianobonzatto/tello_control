#include <thread>
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

using namespace std;

geometry_msgs::Point gotoPose;
geometry_msgs::Pose telloPose;

ros::Publisher pose_pub;

int state;
int pid_flag;
int running;

void pidFlagCallback(const std_msgs::Bool::ConstPtr& msg){
  pid_flag = (int) msg->data;
}

void keyboardThread(){
  char teste[1];

  while(running){
    std::cin >> teste;
    if(teste[0] == 's'){
      state = 1;
    }
    else if(teste[0] == 'l'){
      state = 2;
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_control");
  ros::NodeHandle nh(""), nh_param("~");
  ros::Rate loop_rate(10);

  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher goto_pub;
  ros::Subscriber pid_flag_sub;

  takeoff_pub = nh.advertise<std_msgs::Empty>("/tello/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("/tello/land", 1);
  goto_pub = nh.advertise<std_msgs::Empty>("/pid_control/goto", 1);

  pid_flag_sub = nh.subscribe<std_msgs::Bool>("/pid_control/flag", 1, &pidFlagCallback);

  state = 0;
  running = 1;
  pid_flag = 0;
  std::thread keyboardRead(keyboardThread);


  std_msgs::Empty emptyMsg;

  while(ros::ok()){
    switch(state){
      case 0:
        break;
        
      case 1:
        cout << "caso 1" << endl;
        takeoff_pub.publish(emptyMsg);
        sleep(10);
        state = 0;
        cout << "caso 1 end" << endl;
        break;
        
      case 2:
        cout << "caso 2" << endl;
        land_pub.publish(emptyMsg);
        state = 0;
        cout << "caso 2 end" << endl;
        break;
    }
    loop_rate.sleep();
  }

  running = 0;
}

