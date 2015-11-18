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

//messages
#include <move_base_msgs/MoveBaseAction.h>

//c++ includes

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float currPosX = 0.0 , currPosY = 0.0;

void updateCurrentPose(const move_base_msgs::MoveBaseFeedbackConstPtr &msg) {
  currPosX = (float) msg->base_position.pose.position.x;
  currPosY = (float) msg->base_position.pose.position.y;

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

  //learning_astar object initialization
  learning_astar astar(response);
  ROS_INFO("LRTA* object created successfully");

  //get initial position on rviz
  ros::Subscriber initialSub = nh.subscribe("initialpose", 10, &learning_astar::updateInitialPosition, &astar);
  ROS_INFO("Waiting for the initial pose from RViz");
  while (astar.initialPosition_.header.frame_id.size()==0){
    ros::spinOnce();
  }

  //get the goal on rviz
  ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 10, &learning_astar::updateGoalPosition, &astar);
  ROS_INFO("Waiting for the goal from RViz");
  while (astar.goalPosition_.header.frame_id.size()==0){
    ros::spinOnce();
  }

  //TODO check if the initial and goal positions are valid and ask the user to enter them again if they aren't

                                              //////////////////////////////
                                              // Make a navigation plan  //
                                             //////////////////////////////

  //get a list of waypoints as the plan for turtlebot
  std::vector<geometry_msgs::Pose> wayPoints, sparseWaypoints;
  sparseWaypoints.clear();
  wayPoints = astar.makePlan();

  //to pass every nth waypoint to move_base so that it has a substantial way to go to
  int skipCount = 10, count = 0;

  //last sparseWaypoint should be the goal itself
  geometry_msgs::Pose temp;
  temp.position.x = astar.goalPosition_.pose.position.x;
  temp.position.y = astar.goalPosition_.pose.position.y;
  temp.orientation.w = astar.goalPosition_.pose.orientation.w;
  temp.orientation.z = astar.goalPosition_.pose.orientation.z;
  sparseWaypoints.push_back(temp);

  //get sparse waypoints from dense waypoints
  while(!wayPoints.empty()){
    if(count == skipCount){
      temp = wayPoints.back();
      ROS_INFO("%f, %f", temp.position.x, temp.position.y);
      sparseWaypoints.push_back(temp);
      wayPoints.pop_back();
      count = 0;
    }
    else{
      count++;
      wayPoints.pop_back();
    }
  }

                                          //////////////////////////////////////
                                          // Move  turtlebot as per the plan //
                                          /////////////////////////////////////

  //threshold for goal
  float threshold = 0.3;  //meters

  //setup action client for move_base
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server to come up");
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("...");
    ros::spinOnce();
  }

  ROS_INFO("Client connected");

  //send sparse waypoints to Turtlebot move_base
  while (!sparseWaypoints.empty()){

    geometry_msgs::Pose currentGoal = sparseWaypoints.back();
    sparseWaypoints.pop_back();

    //variable for sending waypoints
    move_base_msgs::MoveBaseGoal goal;

    //create the goal with 'map' frame_id
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = currentGoal.position.x;
    goal.target_pose.pose.position.y = currentGoal.position.y;
    goal.target_pose.pose.orientation.z = astar.initialPosition_.pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = astar.initialPosition_.pose.pose.orientation.w;

    //send the goal
    ROS_INFO("Sending waypoint: %f, %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ac.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(), MoveBaseClient::SimpleActiveCallback(),
                &updateCurrentPose);

    //track the status of goal
    bool atGoal = false;
    while(!atGoal){
      float distance = (float) sqrt((currPosX - currentGoal.position.x)*(currPosX - currentGoal.position.x) + (currPosY - currentGoal.position.y)*(currPosY - currentGoal.position.y));
      if(distance<=threshold){
        atGoal=true;
      }

      ros::spinOnce();
    }
  }

  ros::spin();

  return 0;
}
