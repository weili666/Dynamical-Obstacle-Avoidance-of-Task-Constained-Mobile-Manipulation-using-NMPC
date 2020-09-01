#ifndef _POSITIONALL_H
#define _POSITIONALL_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
using namespace std;
class PositionAll
{
public:
    PositionAll(){}
    ~PositionAll(){}
    bool calcRelativePosition();
    geometry_msgs::Pose people1_pose;
    geometry_msgs::Pose people2_pose;
    geometry_msgs::Pose people3_pose;
    geometry_msgs::Pose people4_pose;
    geometry_msgs::Pose people5_pose;
    geometry_msgs::Pose robot_pose;
    geometry_msgs::Pose arm_pose;
    geometry_msgs::Pose desk_pose;
    geometry_msgs::Pose people1_pose_rel;
    geometry_msgs::Pose people2_pose_rel;
    geometry_msgs::Pose people3_pose_rel;
    geometry_msgs::Pose people4_pose_rel;
    geometry_msgs::Pose people5_pose_rel;
    geometry_msgs::Pose robot_pose_rel;
    geometry_msgs::Pose desk_pose_rel;
};

#endif
