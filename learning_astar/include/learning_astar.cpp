//
// Author: Priyam Parashar
// Date: 11/11/15.
//

#include "learning_astar.h"

learning_astar::learning_astar(){
  mapResolution_ = 0.0;
  mapHeight_ = 0;
  mapWidth_ = 0;
  mapOrigin_ = geometry_msgs::Pose();
  worldMap_ = nav_msgs::OccupancyGrid::();
  dynamicWorldMap = nav_msgs::OccupancyGrid::();
}