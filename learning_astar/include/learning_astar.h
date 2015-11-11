//
// Author: Priyam Parashar
// Date: 11/11/15.
//

//c++ headers
#include <vector>

//ros headers
#include <geometry_msgs/Pose.h>

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

    //all function members
    learning_astar();
    learning_astar(nav_msgs::OccupancyGrid);
    void worldToMap(float wx, float wy, float *mx, float* my); //converts [wx,wy] tp mapCells [mx,my]
    void updateGoalPosition(const geometry_msgs::PoseStampedConstPtr&); //callBack for move_base_simpl/goal
    void updateInitialPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr&); //callback for initialpose

};


#endif //LEARNING_ASTAR_H
