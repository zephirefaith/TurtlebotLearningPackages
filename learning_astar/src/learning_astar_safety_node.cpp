//
// Author: Priyam Parashar 
// Date: 12/8/15
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

//global flags
bool robot_active = true;

//global constants
std::string bot_status[] = {"ACTIVE", "STANDBY"};

//global variables
geometry_msgs::Twist teleop_message;

//callbacks
void robotActiveCallback(const std_msgs::StringConstPtr &msg){

  std::string msg_string = msg->data;
  robot_active = msg_string.compare(bot_status[0]) == 0;
  ROS_INFO("robot is: %s", bot_status[int(robot_active)].c_str());
  return;
}

void teleopCallback(const geometry_msgs::TwistConstPtr &msg){

  teleop_message = *msg;
  return;
}

int main(int argc, char** argv){

  //initialize node
  ros::init(argc, argv, "learning_astar_safety");
  ROS_INFO("LRTA* safety node is up");
  ros::NodeHandle nh;
  ros::Rate r(50);

  //subscribers
  ros::Subscriber robotSub = nh.subscribe("learning_astar/robot_active",5, robotActiveCallback);
  ros::Subscriber teleopSub = nh.subscribe("rms/teleop",5, teleopCallback);

  //publisher
  ros::Publisher teleopPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 50);

  while(ros::ok()){
    if(!robot_active){
      teleopPub.publish(teleop_message);
    }

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}