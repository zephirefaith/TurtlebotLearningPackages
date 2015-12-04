//
// Author: Priyam Parashar
// Date: 11/23/15.
//

//custom includes
#include "../include/learning_astar/learning_astar.h"

//for reading costmap
#include <nav_msgs/GetMap.h>

//for moving the turtlebot
#include <actionlib/client/simple_action_client.h>

//messages
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>

//c++ includes

//typedefs
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//global variables
float currPosX = 0.0, currPosY = 0.0, currOrW = 0.0, currOrZ = 0.0;
bool stuck = false;
bool userActive = false;
int bumperId = -1;

//callbacks

//feedback callback from action client
void updateCurrentPose(const move_base_msgs::MoveBaseFeedbackConstPtr &msg) {
  currPosX = (float) msg->base_position.pose.position.x;
  currPosY = (float) msg->base_position.pose.position.y;
  currOrW = (float) msg->base_position.pose.orientation.w;
  currOrZ = (float) msg->base_position.pose.orientation.z;

  if (userActive) {
    //TODO update the traversal map with ones

  }

  return;
}

//for replan message from rms
void rmsDoneCb(const std_msgs::StringConstPtr &msg){
  userActive = false;

  return;
}

void bumperCb(const kobuki_msgs::BumperEventConstPtr &msg) {
  stuck = true;
  if(bumperId<0){
    bumperId = msg->bumper;
  }
  ROS_INFO("ROBOT BUMPED INTO UNFORESEEN OBSTACLE!!!...%d", bumperId);

  return;
}


//main

int main(int argc, char **argv) {

  ////////////
  // Setup //
  ///////////


  //initialize node
  ros::init(argc, argv, "learning_astar");
  ROS_INFO("LRTA* is running now...");
  ros::NodeHandle nh;

  //publisher topic for RMS calls
  ros::Publisher stuckPub = nh.advertise<std_msgs::String>("robot_stuck", 10);

  //subscriber for RMS feedback
  ros::Subscriber rmsSub = nh.subscribe("user_feedback", 10, &rmsDoneCb);
  //subscriber for bumper event
  ros::Subscriber bumperSub = nh.subscribe("/mobile_base/events/bumper", 50, bumperCb);

  //client for map_server static_map service
  nav_msgs::GetMap mapRequest;
  nav_msgs::OccupancyGrid response;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  //flags
  bool backToTop = false;

  //temp variables
  int updateCount = 0;


  ////////////////////////////////////////
  // Collecting relevant data from ROS //
  ///////////////////////////////////////


  //request for Map
  if (client.call(mapRequest)) {
    ROS_INFO("Received map from the 'static_map' service");
    response = mapRequest.response.map;
  }

  //learning_astar object initialization
  learning_astar astar(response);
  ROS_INFO("LRTA* object created successfully");

  //get initial position on rviz
  ros::Subscriber initialSub = nh.subscribe("initialpose", 10, &learning_astar::updateInitialPosition, &astar);
  ROS_INFO("Waiting for the initial pose from RViz");
  while (astar.initialPosition_.header.frame_id.size() == 0) {
    ros::spinOnce();
  }

  while (ros::ok()) {

    //reset flags
    backToTop = false;
    stuck = false;

    //get the goal on rviz
    if(astar.goalPosition_.header.frame_id.size() == 0){
      ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 10, &learning_astar::updateGoalPosition, &astar);
      ROS_INFO("Waiting for the goal from RViz");
      while (astar.goalPosition_.header.frame_id.size() == 0) {
        ros::spinOnce();
      }
    }



    //////////////////////////////
    // Make a navigation plan  //
    //////////////////////////////


    //get a list of waypoints as the plan for turtlebot
    std::vector<geometry_msgs::Pose> wayPoints, sparseWaypoints;
    sparseWaypoints.clear();
    wayPoints = astar.makePlan();

    //to pass every nth waypoint to move_base so that it has a substantial way to go to
    int skipCount = 3, count = 0;

    //last sparseWaypoint should be the goal itself
    geometry_msgs::Pose temp;
    temp.position.x = astar.goalPosition_.pose.position.x;
    temp.position.y = astar.goalPosition_.pose.position.y;
    temp.orientation.w = astar.goalPosition_.pose.orientation.w;
    temp.orientation.z = astar.goalPosition_.pose.orientation.z;
    sparseWaypoints.push_back(temp);

    //get sparse waypoints from dense waypoints
    while (!wayPoints.empty()) {
      if (count == skipCount) {
        temp = wayPoints.back();
        ROS_INFO("%f, %f", temp.position.x, temp.position.y);
        sparseWaypoints.push_back(temp);
        wayPoints.pop_back();
        count = 0;
      }
      else {
        count++;
        wayPoints.pop_back();
      }
    }


    /////////////////////////////////////
    // Move turtlebot as per the plan //
    ////////////////////////////////////


    //threshold for goal
    float threshold = 0.3;  //meters

    //setup action client for move_base
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    while (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("...");
      ros::spinOnce();
    }

    ROS_INFO("Client connected");

    //send sparse waypoints to Turtlebot move_base
    while (!sparseWaypoints.empty()) {

      geometry_msgs::Pose currentGoal = sparseWaypoints.back();
      sparseWaypoints.pop_back();

      //variable for sending waypoints
      move_base_msgs::MoveBaseGoal goal;

      //create the goal with 'map' frame_id
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = currentGoal.position.x;
      goal.target_pose.pose.position.y = currentGoal.position.y;
      goal.target_pose.pose.orientation.z = astar.goalPosition_.pose.orientation.z;
      goal.target_pose.pose.orientation.w = astar.goalPosition_.pose.orientation.w;

      //send the goal and add a feedback callback
      ROS_INFO("Chasing waypoint: %f, %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
      ac.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(), MoveBaseClient::SimpleActiveCallback(),
                  &updateCurrentPose);
      ROS_INFO("Tracking events now...");

      //track goal progress
      bool atWaypoint = false;

      while (!atWaypoint) {
        float distance = (float) sqrt((currPosX - currentGoal.position.x) * (currPosX - currentGoal.position.x) +
                                      (currPosY - currentGoal.position.y) * (currPosY - currentGoal.position.y));

        //case 1: clear path: check if at waypoint, update flag to send next one
        if (distance <= threshold) {
          atWaypoint = true;
        }

        //case 2: robot gets stuck: cancel goal pursuit -> call RMS -> wait for RMS to exit -> update costs -> get
        // new plan
        if (stuck) {

          ROS_INFO("Engage SOS behaviour...");

          //cancel pursuing of goal
          ROS_INFO("Cancelling goal pursuit and waiting");
          ac.cancelAllGoals();
          ros::Duration(3.0).sleep();

          //store current position, to be used for updating the map
          float stuckX, stuckY, stuckW, stuckZ;
          stuckX = currPosX;
          stuckY = currPosY;
          stuckW = currOrW;
          stuckZ = currOrZ;

          //backup the robot for 0.5 m
          ROS_INFO("Backing up AssFace");
          ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
          geometry_msgs::Twist velocity;
          switch(bumperId){
            case 0:
              velocity.linear.x =
              break;
            case 1:
              velocity.linear.x = -1.0;
              velocity.linear.y = -1.0;
              break;
            case 2:
              break;
          }
          velocity.angular.z=0.0;
          velPub.publish(velocity);

          ros::Duration(1.5).sleep();

          velocity.linear.x = 0.0;
          velocity.linear.y = 0.0;
          velPub.publish(velocity);

          //publish to robot_stuck topic for RMS to take over
          std_msgs::String sosMsg;
          std::stringstream ss;
          ss << "STUCK";
          sosMsg.data = ss.str();
          ROS_INFO("Calling out to RMS");
          stuckPub.publish(sosMsg);
          userActive = true;
          astar.userActive = true;

          //wait for RMS to handover control
          ROS_INFO("Waiting for user to finish helping the bot...");
          ros::Subscriber amclSub = nh.subscribe("amcl_pose", 10, &learning_astar::updateInitialPosition, &astar);
          while (userActive){
            ros::spinOnce();
          }
          astar.userActive = false;

          //update DynamicMap
          ROS_INFO("Gathering relevant info from this interaction...");
          astar.updateDynamicMap(bumperId, stuckX, stuckY, stuckW, stuckZ, updateCount++);
          bumperId = -1;

          //reinitialize the initialPosition for plan making purposes, subscribe to amcl_pose
//          astar.initialPosition_.header.frame_id = std::string();
//          while(astar.initialPosition_.header.frame_id.size()==0){
//            ros::spinOnce();
//          }

          //send back to top to get goal pose from RViz for recomputation of an A* plan
          backToTop = true;
        }

        if (backToTop) {
          break;
        }
        ros::spinOnce();
      }
      if (backToTop) {
        break;
      }
      ros::spinOnce();
    }
    if (!backToTop) {
      ROS_INFO("Successfully reached the goal");
      //cancel goals to make sure move_base isn't still trying to get somewhere it doesn't need to
      ac.cancelAllGoals();
      //reinitialize the initialPosition, for plan making purposes, using the last received updated position of
      // the turtlebot
      astar.initialPosition_.pose.pose.position.x = currPosX;
      astar.initialPosition_.pose.pose.position.y = currPosY;
      astar.initialPosition_.pose.pose.orientation.w = currOrW;
      astar.initialPosition_.pose.pose.orientation.z = currOrZ;

      //update DynamicMap
      ROS_INFO("Updating map post-run");
      astar.updateDynamicMap(bumperId, currPosX, currPosY, currOrW, currOrZ, updateCount++);
      bumperId = -1;

      //reset goal frame_id to get a new one again
      astar.goalPosition_.header.frame_id = std::string();
    }

    ros::spinOnce();
  }

  return 0;
}
