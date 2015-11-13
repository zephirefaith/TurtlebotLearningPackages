//
// Author: Priyam Parashar
// Date: 11/11/15.
//

//custom include
#include "../include/learning_astar/learning_astar.h"


//Constructor just because
learning_astar::learning_astar(){
  mapResolution_ = 0.0;
  mapHeight_ = 0;
  mapWidth_ = 0;
  mapOrigin_ = geometry_msgs::Pose();
  worldMap_ = nav_msgs::OccupancyGrid();
  dynamicWorldMap = nav_msgs::OccupancyGrid();
}

//default Constructor
learning_astar::learning_astar(const nav_msgs::OccupancyGrid mapgrid) {
  worldMap_ = mapgrid;
  dynamicWorldMap = worldMap_;
  mapResolution_ = worldMap_.info.resolution;
  mapOrigin_ = worldMap_.info.origin;
  mapWidth_ = worldMap_.info.width;
  mapHeight_ = worldMap_.info.height;
}

//for updating initial position of the robot
void learning_astar::updateInitialPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  initialPosition_ = *msg;
  ROS_INFO("Received turtlebot's inital position, x: %f, y: %f", initialPosition_.pose.pose.position.x,
           initialPosition_.pose.pose.position.y);

  return;
}

//for updating and storing goal position
void learning_astar::updateGoalPosition(const geometry_msgs::PoseStampedConstPtr &msg) {
  goalPosition_ = *msg;
  ROS_INFO("Received the goal, x: %f, y: %f", goalPosition_.pose.position.x, goalPosition_.pose.position.y);

  return;
}

//for converting from world co-ordinates to map-cells
void learning_astar::worldToMap(float wx, float wy, unsigned int *mx, unsigned int *my) {
  *mx = (unsigned int) ((wx - mapOrigin_.position.x) / mapResolution_);
  *my = (unsigned int) ((wy-mapOrigin_.position.y)/mapResolution_);

  return;
}
