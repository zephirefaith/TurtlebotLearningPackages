//
// Author: Priyam Parashar
// Date: 11/11/15.
//

//custom include
#include "../include/learning_astar/learning_astar.h"

//ros includes


//c++ includes
#include <map>
#include <utility>
#include <queue>


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

//for making a plan using A*
std::vector<geometry_msgs::Pose> learning_astar::makePlan() {
  //get inital mapCells
  unsigned int mx, my;
  worldToMap(mapOrigin_.position.x, mapOrigin_.position.y, &mx, &my);

  //get goal mapCells
  unsigned int gx, gy;
  worldToMap(mapOrigin_.position.x, mapOrigin_.position.y, &gx, &gy);

  //get array of occupancy values from world map
  bool* OGM = new bool [mapHeight_*mapWidth_];
  for (unsigned int iy = 0; iy < mapHeight_; iy++)
  {
    for (unsigned int ix = 0; ix < mapWidth_; ix++)
    {
      unsigned int cost = static_cast<unsigned int>(worldMap_.data[iy*mapWidth_+ix]);
      //cout<<cost;
      if (cost == 0)
        OGM[iy*mapWidth_+ix]=true;
      else
        OGM[iy*mapWidth_+ix]=false;
    }
  }

  //resultant vector with waypoints
  std::vector<geometry_msgs::Pose> wayPoints;
  wayPoints.clear();

  //set up variables: a heap for storing open list sorted on f(), a closed map for looking up if a neighboring cell
  // has already been visited
  std::map< std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int> > closedSet;
  std::priority_queue< std::pair< std::pair<unsigned int, unsigned int>, unsigned int > > openSet;


  return wayPoints;

}