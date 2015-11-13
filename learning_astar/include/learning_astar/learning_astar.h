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

//for moving the turtlebot
#include <actionlib/client/simple_action_client.h>

//messages
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#ifndef LEARNING_ASTAR_H
#define LEARNING_ASTAR_H


class learning_astar
{
public:
    //all variable members
    float mapResolution_;
    int mapWidth_, mapHeight_;
    geometry_msgs::Pose mapOrigin_;
    nav_msgs::OccupancyGrid worldMap_, dynamicWorldMap;
    geometry_msgs::PoseWithCovarianceStamped initialPosition_;
    geometry_msgs::PoseStamped goalPosition_;

    //all function members
    learning_astar();
    learning_astar(const nav_msgs::OccupancyGrid);
    void worldToMap(float wx, float wy, unsigned int *mx, unsigned int *my); //converts [wx,wy] tp mapCells [mx,my]
    void updateGoalPosition(const geometry_msgs::PoseStampedConstPtr&); //callBack for move_base_simpl/goal
    void updateInitialPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr&); //callback for initialpose
    std::vector<geometry_msgs::Pose> makePlan();

};


#endif //LEARNING_ASTAR_H
