//
// Author: Priyam Parashar
// Date: 11/11/15.
//

//c++ headers
#include <vector>

//ROS includes
#include <ros/ros.h>

//for reading costmap
#include <nav_msgs/OccupancyGrid.h>

//messages
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>

#ifndef LEARNING_ASTAR_H
#define LEARNING_ASTAR_H

typedef std::pair<unsigned int, unsigned int> mapCell;

class learning_astar
{
public:
    //all variable members
    bool *OGM;
    float mapResolution_;
    int mapWidth_, mapHeight_;
    int *dynamicWorldMap, *gMap;
    geometry_msgs::Pose mapOrigin_;
    nav_msgs::OccupancyGrid worldMap_;
    geometry_msgs::PoseWithCovarianceStamped initialPosition_;
    geometry_msgs::PoseStamped goalPosition_;

    //all function members
    learning_astar();
    learning_astar(const nav_msgs::OccupancyGrid);
    bool isFree(mapCell);
    int toIndex(mapCell);
    float h(mapCell, mapCell);
    int cost(mapCell, mapCell);
    geometry_msgs::Pose getPose(mapCell);
    std::vector<geometry_msgs::Pose> makePlan();
    void worldToMap(float wx, float wy, unsigned int *mx, unsigned int *my); //converts [wx,wy] to mapCells [mx,my]
    void mapToWorld(unsigned int mx, unsigned int my, float *wx, float *wy); //converts mapCells [mx,my] to [wx,wy]
    void updateGoalPosition(const geometry_msgs::PoseStampedConstPtr&); //callBack for move_base_simple_goal
    void updateInitialPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr&); //callback for initialpose
};


#endif //LEARNING_ASTAR_H
