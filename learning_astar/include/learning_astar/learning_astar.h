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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

#ifndef LEARNING_ASTAR_H
#define LEARNING_ASTAR_H


class learning_astar
{
public:
    //all variable members
    float mapResolution_;
    int mapWidth_, mapHeight_;
    int *dynamicWorldMap, *gMap;
    bool *OGM;
    geometry_msgs::Pose mapOrigin_;
    nav_msgs::OccupancyGrid worldMap_;
    geometry_msgs::PoseWithCovarianceStamped initialPosition_;
    geometry_msgs::PoseStamped goalPosition_;

    //all function members
    learning_astar();
    learning_astar(const nav_msgs::OccupancyGrid);
    void worldToMap(float wx, float wy, unsigned int *mx, unsigned int *my); //converts [wx,wy] to mapCells [mx,my]
    void mapToWorld(unsigned int mx, unsigned int my, float *wx, float *wy); //converts [mx,my] mapCells to world [wx,wy]
    void updateGoalPosition(const geometry_msgs::PoseStampedConstPtr&); //callBack for move_base_simpl/goal
    void updateInitialPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr&); //callback for initialpose
    std::vector<geometry_msgs::Pose> makePlan();  //A* navigation plan
    bool isFree(std::pair<unsigned int, unsigned int>); //to check if a mapCell is free of obstable or not
    int toIndex(std::pair<unsigned int, unsigned int>); //change mapCell co-ords to index
    float h(std::pair<unsigned int, unsigned int>); //heuristic function for A*
    int cost(std::pair<unsigned int, unsigned int> next, std::pair<unsigned int, unsigned int> current); //cost
    // function for getting to next mapCell from current
    bool isGoal(std::pair<unsigned int, unsigned int>); //to check if a given mapCell is the goal //TODO make this personal function of makePlan
    geometry_msgs::Pose getPose(std::pair<unsigned int, unsigned int>); //transform mapCell to geometry_msgs::Pose format
};


#endif //LEARNING_ASTAR_H
