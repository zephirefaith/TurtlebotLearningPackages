//
// Author: Priyam Parashar
// Date: 11/10/15.
//

//ROS includes
#include <ros/ros.h>

//for reading costmap
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>


//for moving the turtlebot
#include <actionlib/client/simple_action_client.h>

//messages
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

//variable to hold the goal position
geometry_msgs::PoseWithCovarianceStamped initialPosition;
geometry_msgs::PoseStamped goalPosition;

void updateGoalPos(const geometry_msgs::PoseStampedConstPtr& msg){
  goalPosition = *msg;
  ROS_INFO("Received the goal, x: %f, y: %f", goalPosition.pose.position.x, goalPosition.pose.position.y);
}

void updateInitialPos(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  initialPosition = *msg;
  ROS_INFO("Received turtlebot's inital position, x: %f, y: %f", initialPosition.pose.pose.position.x,
           initialPosition.pose.pose.position.y);
}

int main(int argc, char **argv){

  //initialize node
  ros::init(argc, argv, "learning_astar");
  ros::NodeHandle nh;

  //variable for sending waypoints
  move_base_msgs::MoveBaseActionGoal goal;

  //get the occupancy grid being published my map_server
  nav_msgs::GetMap mapRequest;
  nav_msgs::OccupancyGrid worldMap, dynamicWorldMap;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  if(client.call(mapRequest)){
    ROS_INFO("Received map from the 'static_map' service");
    worldMap = mapRequest.response.map;
    dynamicWorldMap = worldMap;
  }

  //get the metadata for map
  float mapResolution = worldMap.info.resolution;
  int mapWidth, mapHeight;
  geometry_msgs::Pose mapOrigin = worldMap.info.origin;
  mapWidth = worldMap.info.width;
  mapHeight = worldMap.info.height;

  //get initial position clicked on rviz by user
  ros::Subscriber initialSub = nh.subscribe("initialpose", 10, updateInitialPos);

  //get the goal clicked on rviz by user
  ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 10, updateGoalPos);


  ros::spin();

  return 0;
}