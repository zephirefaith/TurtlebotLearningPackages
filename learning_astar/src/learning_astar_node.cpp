//
// Author: Priyam Parashar
// Date: 11/10/15.
//

//custom includes
#include "../include/learning_astar/learning_astar.h"

//for reading costmap
#include <nav_msgs/GetMap.h>

int main(int argc, char **argv){

  //initialize node
  ros::init(argc, argv, "learning_astar");
  ROS_INFO("LRTA* is running now...");
  ros::NodeHandle nh;

  //variable for sending waypoints
  move_base_msgs::MoveBaseActionGoal goal;

  //get the occupancy grid being published my map_server
  nav_msgs::GetMap mapRequest;
  nav_msgs::OccupancyGrid temp;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  if(client.call(mapRequest)){
    ROS_INFO("Received map from the 'static_map' service");
    temp = mapRequest.response.map;
  }

  //learning astar object initialization
  learning_astar astar(temp);

  //get initial position clicked on rviz by user
  ros::Subscriber initialSub = nh.subscribe("initialpose", 10, &learning_astar::updateInitialPosition, &astar);

  //get the goal clicked on rviz by user
  ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 10, &learning_astar::updateGoalPosition, &astar);

  ros::spin();

  return 0;
}