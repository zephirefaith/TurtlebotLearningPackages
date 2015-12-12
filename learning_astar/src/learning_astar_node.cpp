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
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

//global variables
float currPosX = 0.0, currPosY = 0.0, currOrW = 0.0, currOrZ = 0.0;
bool stuck = false;
bool userActive = false;
int bumperId = -1;

//callbacks
void rmsDoneCb(const std_msgs::StringConstPtr &msg) {
  userActive = false;

  return;
}

void bumperCb(const kobuki_msgs::BumperEventConstPtr &msg) {
  stuck = true;
  if (bumperId < 0) {
    bumperId = msg->bumper;
  }
  ROS_INFO("ROBOT BUMPED INTO UNFORESEEN OBSTACLE!!!...%d", bumperId);

  return;
}

int main(int argc, char **argv) {

  ////////////
  // Setup //
  ///////////


  //initialize node
  ros::init(argc, argv, "learning_astar");
  ROS_INFO("LRTA* is running now...");
  ros::NodeHandle nh;

  //publishers
  ros::Publisher stuckPub = nh.advertise<std_msgs::String>("learning_astar/robot_active", 10);
//  ros::Publisher obstacle = nh.advertise<geometry_msgs::PoseStamped>("learning_astar/obstacle_position", 10);
//  ros::Publisher robotPos = nh.advertise<geometry_msgs::PoseStamped>("learning_astar/robot_stuck_pose", 10);
  ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 100);
  ros::Publisher feedbackPub = nh.advertise<std_msgs::String>("learning_astar/feedback", 10);

  //subscribers
  ros::Subscriber rmsSub = nh.subscribe("user_feedback", 10, rmsDoneCb);
  ros::Subscriber bumperSub = nh.subscribe("mobile_base/events/bumper", 50, bumperCb);

  //client for map_server static_map service
  nav_msgs::GetMap mapRequest;
  nav_msgs::OccupancyGrid response;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  //flags
  bool backToTop = false;
  bool planAbort = false;

  //variables
  int updateCount = 0;
  std_msgs::String sosMsg;
  std::stringstream ss;


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
  ros::Subscriber initialSub = nh.subscribe("initialpose", 10, &learning_astar::updatePosition, &astar);

  //feedback
  ROS_INFO("Waiting for the initial pose from RViz");
  ss.str("");
  ss << "Waiting for the initial pose from RViz";
  sosMsg.data = ss.str();
  feedbackPub.publish(sosMsg);


  while (astar.currentPose_.header.frame_id.size() == 0 && ros::ok()) {
    ros::spinOnce();
  }

  while (ros::ok()) {

    //reset flags
    backToTop = false;
    stuck = false;

    //get the goal on rviz
    if (astar.goalPosition_.header.frame_id.size() == 0  && ros::ok()) {
      ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 10, &learning_astar::updateGoalPosition, &astar);

      //feedback
      ROS_INFO("Waiting for the goal from RViz");
      ss.str("");
      ss << "Waiting for the goal from RViz";
      sosMsg.data = ss.str();
      feedbackPub.publish(sosMsg);

      while (astar.goalPosition_.header.frame_id.size() == 0 && ros::ok()) {
        ros::spinOnce();
      }
    }



    //////////////////////////////
    // Make a navigation plan  //
    //////////////////////////////


    //get a list of waypoints as the plan for turtlebot
    std::vector <geometry_msgs::Pose> wayPoints, sparseWaypoints;
    sparseWaypoints.clear();

    //feedback
    ss.str("");
    ss << "We are building a plan from Start to Goal";
    sosMsg.data = ss.str();
    feedbackPub.publish(sosMsg);

    wayPoints = astar.makePlan();

    //to pass every nth waypoint to move_base so that it has a substantial way to go to
    int skipCount = 2, count = 0;

    //last sparseWaypoint should be the goal itself
    geometry_msgs::Pose temp;
    temp.position.x = astar.goalPosition_.pose.position.x;
    temp.position.y = astar.goalPosition_.pose.position.y;
    temp.orientation.w = astar.goalPosition_.pose.orientation.w;
    temp.orientation.z = astar.goalPosition_.pose.orientation.z;
    sparseWaypoints.push_back(temp);

    //get sparse waypoints from dense waypoints
    while (!wayPoints.empty() && ros::ok()) {
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

    //feedback
    ss.str("");
    ss << "We have a plan! Waiting for robot execution...";
    sosMsg.data = ss.str();
    feedbackPub.publish(sosMsg);

    //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok()) {
      ROS_INFO("...");
      ros::spinOnce();
    }

    ROS_INFO("Client connected");

    //feedback
    ss.str("");
    ss << "And we're moving";
    sosMsg.data = ss.str();
    feedbackPub.publish(sosMsg);

    ros::Subscriber amclSub = nh.subscribe("amcl_pose", 10, &learning_astar::updatePosition, &astar);

    //send sparse waypoints to Turtlebot move_base
    while (!sparseWaypoints.empty() && ros::ok()) {

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
      ac.sendGoal(goal);
      ROS_INFO("Tracking events now...");

      //track goal progress
      bool atWaypoint = false;

      while (!atWaypoint  && ros::ok()) {

        actionlib::SimpleClientGoalState state = ac.getState();
        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
          planAbort = true;
        }

        float distance = (float) sqrt((astar.currentPose_.pose.pose.position.x - currentGoal.position.x) *
                                      (astar.currentPose_.pose.pose.position.x - currentGoal.position.x) +
                                      (astar.currentPose_.pose.pose.position.y - currentGoal.position.y) *
                                      (astar.currentPose_.pose.pose.position.y - currentGoal.position.y));
        //case 1: clear path: check if at waypoint, update flag to send next one
        if (distance <= threshold) {
          atWaypoint = true;
        }

        //case 2: planner aborted due to some problem
        if(planAbort){
          //cancel pursuing of goal
          ROS_INFO("Cancelling goal pursuit and waiting");
          ac.cancelAllGoals();
          ros::Duration(0.5).sleep();

          //feedback
          ss.str("");
          ss << "Planner could not find a path for the robot. Please steer it to a more open space so that it's "
                    "easier to replan.";
          sosMsg.data = ss.str();
          feedbackPub.publish(sosMsg);

          //ask for help from rms
          //publish to robot_active topic for RMS to take over
          ss.str("");
          ss << "STANDBY";
          sosMsg.data = ss.str();
          ROS_INFO("Calling out to RMS");
          for (int i = 0; i < 5; ++i) {
            stuckPub.publish(sosMsg);
          }
          userActive = true;
          astar.userActive = true;

          //get control back from rms
          //wait for RMS to handover control
          ROS_INFO("Waiting for user to finish helping the bot...");
          while (userActive) {
            ros::spinOnce();
          }
          astar.userActive = false;

          //send control to replan
          backToTop = true;
          planAbort = false;
        }

        //case 3: robot gets stuck: cancel goal pursuit -> call RMS -> wait for RMS to exit -> update costs -> get
        // new plan
        if (stuck) {

          //feedback
          ss.str("");
          ss << "The bot has hit an obstacle. Please help the bot get around the obstacle. Directions are on left of "
                    "the screen.";
          sosMsg.data = ss.str();
          feedbackPub.publish(sosMsg);

          ROS_INFO("Engage SOS behaviour...");
          //store current position, to be used for updating the map
          float stuckX, stuckY, stuckW, stuckZ;
          stuckX = (float) astar.currentPose_.pose.pose.position.x;
          stuckY = (float) astar.currentPose_.pose.pose.position.y;
          stuckW = (float) astar.currentPose_.pose.pose.orientation.w;
          stuckZ = (float) astar.currentPose_.pose.pose.orientation.z;

          //cancel pursuing of goal
          ROS_INFO("Cancelling goal pursuit and waiting");
          ac.cancelAllGoals();
          ros::Duration(0.5).sleep();
          astar.getObstaclePosition(bumperId);

          //backup the robot
          ROS_INFO("Backing up AssFace");
          geometry_msgs::Twist velocity;
          velocity.linear.x = -0.5;
          velocity.angular.z = 0.0;
          count = 0;
          while(count++<8){
            velPub.publish(velocity);
            ros::Duration(0.1).sleep();
          }
          velocity.linear.x = 0.0;
          velocity.linear.y = 0.0;
          count = 0;
          while(count++<3) {
            velPub.publish(velocity);
            ros::Duration(0.1).sleep();
          }

          //publish to robot_active topic for RMS to take over
          ss.str("");
          ss << "STANDBY";
          sosMsg.data = ss.str();
          ROS_INFO("Calling out to RMS");
          for (int i = 0; i < 5; ++i) {
            stuckPub.publish(sosMsg);
          }
          userActive = true;
          astar.userActive = true;

          //wait for RMS to handover control
          ROS_INFO("Waiting for user to finish helping the bot...");
          while (userActive) {
            ros::spinOnce();
          }
          astar.userActive = false;

          //publish to robot_active that robot is driving autonomously now
          ss.str("");
          ss << "ACTIVE";
          sosMsg.data = ss.str();
          for (int i = 0; i < 5; ++i) {
            stuckPub.publish(sosMsg);
          }

          //update DynamicMap
          ROS_INFO("Gathering relevant info from this interaction...");
          std::vector<float> obstaclePos;
          obstaclePos = astar.updateDynamicMap(bumperId, updateCount++);
          bumperId = -1;

//          //publish obstacle position and robot's "stuck" position with the robot's captured orientation
//          geometry_msgs::PoseStamped genericPose;
//          genericPose.header.frame_id = "map";
//          genericPose.pose.position.y = obstaclePos[1];
//          genericPose.pose.position.x = obstaclePos[0];
//          genericPose.pose.orientation.w = stuckW;
//          genericPose.pose.orientation.z = stuckZ;
////          obstacle.publish(genericPose);
//          genericPose.pose.position.y = stuckY;
//          genericPose.pose.position.x = stuckX;
////          robotPos.publish(genericPose);

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

      //feedback
      ss.str("");
      ss << "Yay! We reached the goal!";
      sosMsg.data = ss.str();
      feedbackPub.publish(sosMsg);

      //update DynamicMap
      ROS_INFO("Updating map post-run");
      astar.updateDynamicMap(bumperId, updateCount++);
      bumperId = -1;

      //reset goal frame_id to get a new one again
      astar.goalPosition_.header.frame_id = std::string();
    }

    ros::spinOnce();
  }

  return 0;
}
