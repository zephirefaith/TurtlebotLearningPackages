//
// Author: Priyam Parashar
// 11/10/15.
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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


int main(int argc, char **argv){

  //initialize node
  ros::init(argc, argv, "learning_astar");
  ros::NodeHandle nh;

  //get the occupancy grid being published my map_server
  nav_msgs::GetMap mapRequest;
  nav_msgs::OccupancyGrid worldMap, dynamicWorldMap;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  if(client.call(mapRequest)){
    ROS_INFO("Received map from the 'static_map' service");
    worldMap = mapRequest.response.map;
    dynamicWorldMap = worldMap;
  }

  return 0;
}

