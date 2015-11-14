//
// Author: Priyam Parashar
// Date: 11/10/15.
//

//custom includes
#include "../include/learning_astar/learning_astar.h"

//for reading costmap
#include <nav_msgs/GetMap.h>

//for moving the turtlebot
#include <actionlib/client/simple_action_client.h>

//c++ includes
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float currX = 0.0 , currY = 0.0;

void updateCurrentPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  currX = msg->pose.pose.position.x;
  currY = msg->pose.pose.position.y;

  return;
}

int main(int argc, char **argv){
                                                        ////////////
                                                        // Setup //
                                                        ///////////
  //initialize node
  ros::init(argc, argv, "learning_astar");
  ROS_INFO("LRTA* is running now...");
  ros::NodeHandle nh;

                                          ////////////////////////////////////////
                                          // Collecting relevant data from ROS //
                                          ///////////////////////////////////////

  //get the occupancy grid being published my map_server
  nav_msgs::GetMap mapRequest;
  nav_msgs::OccupancyGrid response;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  //get the map response
  if(client.call(mapRequest)){
    ROS_INFO("Received map from the 'static_map' service");
    response = mapRequest.response.map;
  }

  //learning astar object initialization
  learning_astar astar(response);

  //get initial position clicked on rviz by user
  ros::Subscriber initialSub = nh.subscribe("initialpose", 10, &learning_astar::updateInitialPosition, &astar);

  //get the goal clicked on rviz by user
  ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 10, &learning_astar::updateGoalPosition, &astar);

  //TODO check if the initial and goal positions are valid and ask the user to enter them again if they aren't

                                              //////////////////////////////
                                              // Make a navigation plan  //
                                             //////////////////////////////

  //get a list of waypoints as the plan for turtlebot
  std::vector<geometry_msgs::Pose> wayPoints, sparseWaypoints;
  sparseWaypoints.clear();
  wayPoints = astar.makePlan();

  //to pass every nth waypoint to move_base so that it has a substantial way to go to
  int skipCount = 15, count = 0;

  //get sparse waypoints from dense waypoints
  geometry_msgs::Pose temp;
  while(!wayPoints.empty()){
    if(count == skipCount){
      sparseWaypoints.push_back(wayPoints.back());
      wayPoints.pop_back();
      count = 0;
    }
    else{
      count++;
      temp = wayPoints.back();
      wayPoints.pop_back();
    }
  }

  //if the last wayPoint was not added to sparseWaypoints, add it
  if(count<skipCount){
    sparseWaypoints.push_back(temp);
  }

                                          //////////////////////////////////////
                                          // Move  turtlebot as per the plan //
                                          /////////////////////////////////////

  //threshold for goal
  float threshold = 0.25;  //meters

  //setup action client for move_base
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //variable for sending waypoints
  move_base_msgs::MoveBaseGoal goal;

  //send sparse waypoints to Turtlebot move_base
  while (!sparseWaypoints.empty()){

    geometry_msgs::Pose current = sparseWaypoints.back();
    sparseWaypoints.pop_back();

    //create the goal with 'map' frame_id
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = current.position.x;
    goal.target_pose.pose.position.y = current.position.y;

    //send the goal
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    //track the status of goal
    bool atGoal = false;
    while(!atGoal){
      //subscribe to amcl_pose and check if within threshold
      ros::Subscriber amclSub = nh.subscribe("amcl_pose", 10, updateCurrentPose);
      float distance = sqrt(pow(currX-current.position.x, 2) + pow(currY-current.position.y, 2));
      if(distance<=threshold){
        atGoal=true;
      }

      ros::spinOnce();
    }
  }

  ros::spin();

  return 0;
}