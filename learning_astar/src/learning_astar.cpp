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
#include <limits.h>
#include <algorithm>
#include <math.h>
#include <iostream>

typedef std::pair<unsigned int, unsigned int> mapCell;

//class comparator for minHeap of mapCells
class myComparator{
public:
    bool operator() (std::pair<mapCell, float> a, std::pair<mapCell, float> b){
      return a.second < b.second;
    }
};

//Constructor just because
learning_astar::learning_astar(){
  mapResolution_ = 0.0;
  mapHeight_ = 0;
  mapWidth_ = 0;
  mapOrigin_ = geometry_msgs::Pose();
  worldMap_ = nav_msgs::OccupancyGrid();
  dynamicWorldMap = new float;
  gMap = new float;
  OGM = new bool;
}

//default Constructor
learning_astar::learning_astar(const nav_msgs::OccupancyGrid mapgrid) {
  worldMap_ = mapgrid;
  mapResolution_ = worldMap_.info.resolution;
  mapOrigin_ = worldMap_.info.origin;
  mapWidth_ = worldMap_.info.width;
  mapHeight_ = worldMap_.info.height;

  //initialize dynamic map to zero
  dynamicWorldMap = new float[mapHeight_*mapWidth_];
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      dynamicWorldMap[y*mapWidth_+x] = 0;
    }
  }

  //initialize static map to highest value everywhere
  gMap = new float[mapHeight_*mapWidth_];
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      gMap[y*mapWidth_+x] = 255;
    }
  }

  //get array of occupancy values from world map
  OGM = new bool [mapHeight_*mapWidth_];
  for (unsigned int iy = 0; iy < mapHeight_; iy++)
  {
    for (unsigned int ix = 0; ix < mapWidth_; ix++)
    {
      unsigned int cost = static_cast<unsigned int>(worldMap_.data[iy*mapWidth_+ix]);  //map is in row major form,
      // hence iy*mapWidth_
      //cout<<cost;
      OGM[iy*mapWidth_+ix]= !cost;
    }
  }

//  ROS_INFO("MapHeight: %d, MapWidth: %d", mapHeight_, mapWidth_);
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
  *my = (unsigned int) ((wy - mapOrigin_.position.y)/mapResolution_);

  return;
}

//change from map to world co-ordinates
void learning_astar::mapToWorld(unsigned int mx, unsigned int my, float *wx, float *wy) {
  *wx = (float) (mx*mapResolution_*1.0 + mapOrigin_.position.x);
  *wy = (float) (my*mapResolution_*1.0 + mapOrigin_.position.y);

  return;
}

//for converting from mapCell to index
int learning_astar::toIndex(mapCell cell) {
  int index = cell.second*mapWidth_+cell.first;
  return index;
}

//heuristic function
float learning_astar::h(mapCell current, mapCell goal) {
  float hScore = sqrt((current.first-goal.first)^2+(current.second-goal.second)^2);

  return hScore;
}

//for checking if a mapCell is free to move to
bool learning_astar::isFree(mapCell cell) {
  return OGM[toIndex(cell)];
}

//for calculating cost of travelling from one mapCell to other
int learning_astar::cost(mapCell current, mapCell next) {
  int hopCost = abs(current.first-next.first)+abs(current.second-next.second);
//  ROS_INFO("%d,%d to %d,%d: %d", current.first,current.second,next.first,next.second, hopCost);
  return hopCost;
}

//for converting mapCells to Pose format
geometry_msgs::Pose learning_astar::getPose(mapCell cell) {
  geometry_msgs::Pose wayPoint;
  float wx, wy;
  mapToWorld(cell.first, cell.second, &wx, &wy);
  wayPoint.position.x = wx;
  wayPoint.position.y = wy;

  return wayPoint;
}

//for making a plan using A*
std::vector<geometry_msgs::Pose> learning_astar::makePlan() {

  //for neighbors
  int dx[3] = {-1, 0,1 };
  int dy[3] = {-1, 0, 1};

  //get initial location
  unsigned int mx, my;
  worldToMap(initialPosition_.pose.pose.position.x, initialPosition_.pose.pose.position.y, &mx, &my);
  ROS_INFO("Start on map: (%d, %d)", mx, my);

  //get goal location
  unsigned int gx, gy;
  worldToMap(goalPosition_.pose.position.x, goalPosition_.pose.position.y, &gx, &gy);
  ROS_INFO("Goal on map: (%d, %d)", gx, gy);

  //store initial and goal position
  mapCell initialPose = std::make_pair(mx,my), goalPose = std::make_pair(gx,gy);

  //resultant vector with waypoints
  std::vector<geometry_msgs::Pose> wayPoints;
  wayPoints.clear();

  //flag for whether goal is found or not
  bool goalFound = false;

  //initialize gValues to highest value everywhere, except for the initial position, that should be 0
  int initialIndex = toIndex(initialPose);
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      gMap[y*mapWidth_+x] = 255;
    }
  }
  gMap[initialIndex] = 0;

  //set up variables: a heap for storing open list sorted on increasing f(), a closed map for looking up if a
  // neighboring cell has already been visited
  std::map< mapCell, mapCell > closedSet, openSet, parent;
  std::priority_queue< std::pair< mapCell, float >, std::vector< std::pair< mapCell, float > >, myComparator > openList;
  mapCell current;

  ROS_INFO("Finding an A* plan...");
  current = initialPose;
  openList.push(std::make_pair(current, 0));
  while(!openList.empty() && !goalFound){

    current = openList.top().first;
    openList.pop();
    closedSet[current] = current;
    int current_cell = toIndex(current);
    //find neighbors of current, you can move in any of the 8 adjacent boxes as long as they are free
    for (int i = 0; i < 3 && !goalFound; ++i) {
      for (int j = 0; j < 3 && !goalFound; ++j) {

        unsigned int nx = current.first+dx[i], ny = current.second+dy[j];
        mapCell neighbor = std::make_pair(nx,ny);

        if (isFree(neighbor)) {
          //check if the neighboring cell is free and has not been explored already
          std::map<mapCell, mapCell>::iterator closedIt = closedSet.find(neighbor), openIt = openSet.find(neighbor);
          if (closedIt == closedSet.end()) {

            int neighbor_cell = toIndex(neighbor);
            float newg = gMap[current_cell] + (float)(cost(current, neighbor));
            if(openIt == openSet.end()) {
              //a new node
              //update
              gMap[neighbor_cell] = newg;

              //calculate f
              float f_cell = gMap[neighbor_cell] + dynamicWorldMap[neighbor_cell] + h(neighbor, goalPose);

              //push onto the heap and openSet map
              openList.push(std::make_pair(neighbor, f_cell));
              openSet[neighbor] = neighbor;

              //store the parent of this mapCell
              parent[neighbor] = current;

              //check if neighbor is the goal, if yes: update the flag
              if (neighbor.first == goalPose.first && neighbor.second == goalPose.second) {
                current = neighbor;
                goalFound = true;

              }
            }else if (gMap[neighbor_cell] > newg) {

              //update
              gMap[neighbor_cell] = newg;

              //calculate f
              float f_cell = gMap[neighbor_cell] + dynamicWorldMap[neighbor_cell] + h(neighbor, goalPose);

              //push onto the heap and openSet map
              openList.push(std::make_pair(neighbor, f_cell));

              //store the parent of this mapCell
              parent[neighbor] = current;

              //check if neighbor is the goal, if yes: update the flag
              if (neighbor.first == goalPose.first && neighbor.second == goalPose.second) {
                current = neighbor;
                goalFound = true;

              }

            }

          }
        }

      }
    }

//    getchar();
  }

  ROS_INFO("Found a plan...creating waypoints");

  //if goal was found, backtrack to get the waypoints or just return an empty waypoints vector
  if(goalFound){

    //push back parents until map has no more
    std::map< mapCell, mapCell>::iterator mapIt = parent.find(current);
    while(mapIt!=parent.end()) {
      ROS_INFO("Map Cell: %d, %d", current.first, current.second);
      wayPoints.push_back(getPose(current));
      current = parent[current];
      mapIt = parent.find(current);
    }
    wayPoints.push_back(getPose(current));
    std::reverse(wayPoints.begin(), wayPoints.end());
  }

  return wayPoints;
}
