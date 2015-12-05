//
// Author: Priyam Parashar
// Date: 11/23/15.
//

//custom includes
#include "../include/learning_astar/learning_astar.h"

//ros includes
#include <tf/LinearMath/Quaternion.h>

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//c++ includes
#include <queue>
#include <fstream>

typedef std::pair<unsigned int, unsigned int> mapCell;

//Constructor just because
learning_astar::learning_astar() {
  mapResolution_ = 0.0;
  mapHeight_ = 0;
  mapWidth_ = 0;
  mapOrigin_ = geometry_msgs::Pose();
  worldMap_ = nav_msgs::OccupancyGrid();
  dynamicWorldMap = new float;
  traversalMap = new float;
  OGM = new bool;
  botRadius = 0.0;
}

//default Constructor
learning_astar::learning_astar(const nav_msgs::OccupancyGrid mapgrid) {
  softObstacleX = -1;
  softObstacleY = -1;
  sigma = 0.8;
  botRadius = 0.2;
  incrementConstant = 300;
  decayConstant = 0.6;
  worldMap_ = mapgrid;
  mapResolution_ = worldMap_.info.resolution;
  mapOrigin_ = worldMap_.info.origin;
  mapWidth_ = worldMap_.info.width;
  mapHeight_ = worldMap_.info.height;
  userActive = false;

  //initialize dynamic map to zero
  dynamicWorldMap = new float[mapHeight_ * mapWidth_];
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      dynamicWorldMap[y * mapWidth_ + x] = 0.0;
    }
  }

  //initialize traversal map to all zero
  traversalMap = new float[mapHeight_ * mapWidth_];
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      traversalMap[y * mapWidth_ + x] = 0.0;
    }
  }

  //get array of occupancy values from world map
  OGM = new bool[mapHeight_ * mapWidth_];
  for (unsigned int iy = 0; iy < mapHeight_; iy++) {
    for (unsigned int ix = 0; ix < mapWidth_; ix++) {
      unsigned int cost = static_cast<unsigned int>(worldMap_.data[iy * mapWidth_ + ix]);  //map is in row major form,
                                                                                          // hence iy*mapWidth_
      OGM[iy * mapWidth_ + ix] = cost==0;
    }
  }

  //  //dumping the map into a CSV, to be visualized by matlab
  std::ofstream ofs;
  ofs.open("/home/priyamp/Documents/Codebase/MapData/StaticMap.csv", std::ofstream::out);

  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      ofs << OGM[y * mapWidth_ + x];
      if (y < mapHeight_ - 1) {
        ofs << ",";
      }
    }
    ofs << std::endl;
  }

  ofs.close();

  //TODO deal with later
//  //inflate temp
//  cv::Mat mapImage(mapHeight_,mapWidth_, CV_32FC1, temp), dilatedMap(mapHeight_,mapWidth_, CV_32FC1);
//  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
//  cv::dilate(mapImage, dilatedMap, element);
//
//  OGM = new bool[mapHeight_ * mapWidth_];
//  for (unsigned int iy = 0; iy < mapHeight_; iy++) {
//    for (unsigned int ix = 0; ix < mapWidth_; ix++) {
//      OGM[iy * mapWidth_ + ix] = dilatedMap.at<float>(ix, iy) == 0.0;
//    }
//  }
//
//  //dumping the map into a CSV, to be visualized by matlab
//  std::ofstream ofs;
//  ofs.open("/home/priyamp/Documents/Codebase/MapData/temp.csv", std::ofstream::out);
//
//  for (int x = 0; x < mapWidth_; ++x) {
//    for (int y = 0; y < mapHeight_; ++y) {
//      ofs << temp[y * mapWidth_ + x];
//      if (y < mapHeight_ - 1) {
//        ofs << ",";
//      }
//    }
//    ofs << std::endl;
//  }
//
//  ofs.close();
//
//  //dumping the map into a CSV, to be visualized by matlab
//  ofs.open("/home/priyamp/Documents/Codebase/MapData/Erodedtemp.csv", std::ofstream::out);
//
//  for (int x = 0; x < mapWidth_; ++x) {
//    for (int y = 0; y < mapHeight_; ++y) {
//      ofs << dilatedMap.at<float>(x,y);
//      if (y < mapHeight_ - 1) {
//        ofs << ",";
//      }
//    }
//    ofs << std::endl;
//  }
//
//  ofs.close();
//  ROS_INFO("MapHeight: %d, MapWidth: %d", mapHeight_, mapWidth_);
}

//for updating initial position of the robot
void learning_astar::updatePosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  currentPose_ = *msg;

  if(userActive){
    unsigned int mx,my;
    worldToMap((float) currentPose_.pose.pose.position.x, (float) currentPose_.pose.pose.position.y, &mx, &my);
    traversalMap[my*mapWidth_+mx] = 50;
  }

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
  *my = (unsigned int) ((wy - mapOrigin_.position.y) / mapResolution_);

  return;
}

//change from map to world co-ordinates
void learning_astar::mapToWorld(unsigned int mx, unsigned int my, float *wx, float *wy) {
  *wx = (float) (mx * mapResolution_ * 1.0 + mapOrigin_.position.x);
  *wy = (float) (my * mapResolution_ * 1.0 + mapOrigin_.position.y);

  return;
}

//for converting from mapCell to index
int learning_astar::toIndex(mapCell cell) {
  int index = cell.second * mapWidth_ + cell.first;
  return index;
}

//heuristic function
float learning_astar::h(mapCell current, mapCell goal) {
  float hScore = (float) sqrt((current.first - goal.first) * (current.first - goal.first) +
                              (current.second - goal.second) * (current.second - goal.second));

  return hScore;
}

//for checking if a mapCell is free to move to
bool learning_astar::isFree(mapCell cell) {
  return OGM[toIndex(cell)];
}

//for calculating cost of travelling from one mapCell to other
float learning_astar::cost(mapCell current, mapCell next) {
  float hopCost = (float) sqrt(
      (current.first - next.first) * (current.first - next.first) + (current.second - next.second) * (current
                                                                                                          .second -
                                                                                                      next.second));

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

//a 2d gaussian
float learning_astar::gaussian2d(int x, int y) {
  if (softObstacleX < 0 || softObstacleY < 0)
    return 0.0;

  float gauss_value = (float) (incrementConstant *
                               exp(-((x - softObstacleX) * (x - softObstacleX) + (y - softObstacleY) * (y -
                                                                                                        softObstacleY)) /
                                   (2 * sigma *
                                    sigma)));

  return gauss_value;
}

//for making a plan using A*
std::vector<geometry_msgs::Pose> learning_astar::makePlan() {

  //for neighbors
  int dx[3] = {-1, 0, 1};
  int dy[3] = {-1, 0, 1};

  //get initial location
  unsigned int mx, my;
  worldToMap((float) currentPose_.pose.pose.position.x, (float) currentPose_.pose.pose.position.y, &mx, &my);
  ROS_INFO("Start on map: (%d, %d)", mx, my);

  //get goal location
  unsigned int gx, gy;
  worldToMap((float) goalPosition_.pose.position.x, (float) goalPosition_.pose.position.y, &gx, &gy);
  ROS_INFO("Goal on map: (%d, %d)", gx, gy);

  //store initial and goal position
  mapCell initialPose = std::make_pair(mx, my), goalPose = std::make_pair(gx, gy);

  //resultant vector with waypoints
  std::vector<geometry_msgs::Pose> wayPoints;
  wayPoints.clear();

  //flag for whether goal is found or not
  bool goalFound = false;

  //initialize gValues to highest value everywhere, except for the initial position, that should be 0
  float *gMap = new float[mapHeight_ * mapWidth_];
  int initialIndex = toIndex(initialPose);
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      gMap[y * mapWidth_ + x] = FLT_MAX;
    }
  }
  gMap[initialIndex] = 0;

  //reset traversal values to zero
  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      traversalMap[y * mapWidth_ + x] = 0.0;
    }
  }

  //set up variables: a heap for storing open list sorted on increasing f(), a closed map for looking up if a
  // neighboring cell has already been visited
  std::map<mapCell, mapCell> closedSet, openSet, parent;
  std::map<float, mapCell> openList;
  mapCell current;

  ROS_INFO("Finding an A* plan...");
  current = initialPose;
  openList[0] = current;
  while (!openList.empty() && !goalFound) {

    current = openList.begin()->second;
    openList.erase(openList.begin());
    closedSet[current] = current;
    //ROS_INFO("Current cell: %d,%d", current.first, current.second);
    int current_cell = toIndex(current);
    //find neighbors of current, you can move in any of the 8 adjacent boxes as long as they are free
    for (int i = 0; i < 3 && !goalFound; ++i) {
      for (int j = 0; j < 3 && !goalFound; ++j) {

        unsigned int nx = current.first + dx[i], ny = current.second + dy[j];
        mapCell neighbor = std::make_pair(nx, ny);

        if (isFree(neighbor)) {
          //check if the neighboring cell is free and has not been explored already
          std::map<mapCell, mapCell>::iterator closedIt = closedSet.find(neighbor), openIt = openSet.find(neighbor);
          if (closedIt == closedSet.end()) {

            int neighbor_cell = toIndex(neighbor);
            float newg = gMap[current_cell] + (cost(current, neighbor));
            //ROS_INFO("Evaluating neighbor: %d,%d", neighbor.first, neighbor.second);
            if (gMap[neighbor_cell] > newg) {

              //ROS_INFO("NewG is less than olderG: %f < %f", newg, gMap[neighbor_cell]);

              //update
              gMap[neighbor_cell] = newg;

              //calculate f
              float f_value = gMap[neighbor_cell] + dynamicWorldMap[neighbor_cell] + h(neighbor, goalPose);

              //push onto the heap and openSet map
              openList[f_value] = neighbor;
              //ROS_INFO("FValue(%d, %d): %f", neighbor.first,neighbor.second, f_value);

              //store the parent of this mapCell
              parent[neighbor] = current;

              //check if neighbor is the goal, if yes: update the flag
              if (neighbor.first == goalPose.first && neighbor.second == goalPose.second) {
                current = neighbor;
                goalFound = true;

              }

              if (openIt == openSet.end()) {
                openSet[neighbor] = neighbor;
              }
            }
          }

        }
      }

    }
    //getchar();
  }


  //if goal was found, backtrack to get the waypoints or just return an empty waypoints vector
  if (goalFound) {

    ROS_INFO("Found a plan...creating waypoints");

    //push back parents until map has no more
    std::map<mapCell, mapCell>::iterator mapIt = parent.find(current);
    while (mapIt != parent.end()) {
      wayPoints.push_back(getPose(current));
      current = parent[current];
      mapIt = parent.find(current);
    }
    wayPoints.push_back(getPose(current));
    std::reverse(wayPoints.begin(), wayPoints.end());
  }

  return wayPoints;
}

//for updating Dynamic Map based on bumper sensors
void learning_astar::updateDynamicMap(int bumperId, float x, float y, float w, float z, int updateCount) {
  //inflate the traversal map using openCV
  cv::Mat mapImage(mapHeight_,mapWidth_, CV_32FC1, traversalMap), dilatedMap(mapHeight_,mapWidth_, CV_32FC1);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::dilate(mapImage, dilatedMap, element);

  //reset soft obstacle position
  softObstacleX = -1;
  softObstacleY = -1;

  //variables for storing the position where cost is to be increased
  float wx = 0.0, wy = 0.0;

  //create Quaternion
  tf::Quaternion botQt;
  botQt.setW(w);
  botQt.setZ(z);

  //get angle
  float radianAngle = (float) botQt.getAngle();

  //depending upon bumperId, get perimeter points (in world frame) where we need to increase the cost
  ROS_INFO("BumperEvent from Bumper: %d", bumperId);
  switch (bumperId) {
    case -1:
      //no collision, hence no need to do anything
      break;
    case 0:
      wx = (float) (x + botRadius * cos(radianAngle+0.78));
      wy = (float) (y + botRadius * sin(radianAngle+0.78));
      break;
    case 1:
      wx = (float) (x + botRadius * cos(radianAngle));
      wy = (float) (y + botRadius * sin(radianAngle));
      break;
    case 2:
      wx = (float) (x + botRadius * cos(radianAngle-0.78));
      wy = (float) (y + botRadius * sin(radianAngle-0.78));
      break;
  }
//  wx = (float) (x - botRadius * cos(radianAngle));
//  wy = (float) (y - botRadius * sin(radianAngle));

  //convert to mapCells, only if there was a collision
  unsigned int mx, my;
  if (bumperId >= 0) {
    worldToMap(wx, wy, &mx, &my);
    softObstacleX = mx;
    softObstacleY = my;
    worldToMap(x,y,&mx,&my);
  }

  //change the dynamic Map, do anyways, because the map should decay to all zeros, even not bumping into anything is
  // an experience...also clear out the places the robot believes it has traversed
  for (int i = 0; i < mapWidth_; ++i) {
    for (int j = 0; j < mapHeight_; ++j) {
      if (dilatedMap.at<float>(i,j)==0.0) {
        dynamicWorldMap[j * mapWidth_ + i] =
            decayConstant * gaussian2d(i, j) + (1 - decayConstant) * dynamicWorldMap[j * mapWidth_ + i];
      } else {
        dynamicWorldMap[j*mapHeight_+i] = 0;
      }
    }
  }

  ROS_INFO("Updated map...resetting and storing state");

  //reset the traversalMap to all zeros
  std::fill_n(traversalMap, mapHeight_*mapWidth_, 0.0);

  //dumping the dynamic map into a CSV, to be visualized by matlab
  std::ofstream ofs;
  std::stringstream ss;
  ss << "/home/priyamp/Documents/Codebase/MapData/DynamicMapStages/DynamicMap" << updateCount << ".csv";
  ofs.open(ss.str().c_str(), std::ofstream::out);

  for (int x = 0; x < mapWidth_; ++x) {
    for (int y = 0; y < mapHeight_; ++y) {
      ofs << dynamicWorldMap[y * mapWidth_ + x];
      if (y < mapHeight_ - 1) {
        ofs << ",";
      }
    };
    ofs << std::endl;
  }

  ofs.close();

  return;
}
