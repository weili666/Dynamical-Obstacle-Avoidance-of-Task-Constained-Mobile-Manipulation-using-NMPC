#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <PositionAll.h>
using namespace std;

bool PositionAll::calcRelativePosition()
{
    double theta1=2*asin(robot_pose.orientation.z);
    double theta2_desk=2*asin(desk_pose.orientation.z);
    double thetar_desk=theta2_desk-theta1;
    robot_pose_rel.position.x=0;
    robot_pose_rel.position.y=0;
    robot_pose_rel.position.z=robot_pose.position.z;
    robot_pose_rel.orientation.w=1;
    robot_pose_rel.orientation.x=0;
    robot_pose_rel.orientation.y=0;
    robot_pose_rel.orientation.z=0;

    desk_pose_rel.position.x=(desk_pose.position.x-robot_pose.position.x)*cos(theta1)+(desk_pose.position.y-robot_pose.position.y)*sin(theta1);
    desk_pose_rel.position.y=(robot_pose.position.x-desk_pose.position.x)*sin(theta1)+(desk_pose.position.y-robot_pose.position.y)*cos(theta1);
    desk_pose_rel.position.z=desk_pose.position.z-robot_pose.position.z;
    desk_pose_rel.orientation.w=cos(thetar_desk/2.0);
    desk_pose_rel.orientation.x=0;
    desk_pose_rel.orientation.y=0;
    desk_pose_rel.orientation.z=sin(thetar_desk/2.0);

    people1_pose_rel.position.x=(people1_pose.position.x-robot_pose.position.x)*cos(theta1)+(people1_pose.position.y-robot_pose.position.y)*sin(theta1);
    people1_pose_rel.position.y=(robot_pose.position.x-people1_pose.position.x)*sin(theta1)+(people1_pose.position.y-robot_pose.position.y)*cos(theta1);
    people1_pose_rel.position.z=people1_pose.position.z-robot_pose.position.z;
    people1_pose_rel.orientation.w=1;
    people1_pose_rel.orientation.x=0;
    people1_pose_rel.orientation.y=0;
    people1_pose_rel.orientation.z=0;

    people2_pose_rel.position.x=(people2_pose.position.x-robot_pose.position.x)*cos(theta1)+(people2_pose.position.y-robot_pose.position.y)*sin(theta1);
    people2_pose_rel.position.y=(robot_pose.position.x-people2_pose.position.x)*sin(theta1)+(people2_pose.position.y-robot_pose.position.y)*cos(theta1);
    people2_pose_rel.position.z=people2_pose.position.z-robot_pose.position.z;
    people2_pose_rel.orientation.w=1;
    people2_pose_rel.orientation.x=0;
    people2_pose_rel.orientation.y=0;
    people2_pose_rel.orientation.z=0;

    people3_pose_rel.position.x=(people3_pose.position.x-robot_pose.position.x)*cos(theta1)+(people3_pose.position.y-robot_pose.position.y)*sin(theta1);
    people3_pose_rel.position.y=(robot_pose.position.x-people3_pose.position.x)*sin(theta1)+(people3_pose.position.y-robot_pose.position.y)*cos(theta1);
    people3_pose_rel.position.z=people3_pose.position.z-robot_pose.position.z;
    people3_pose_rel.orientation.w=1;
    people3_pose_rel.orientation.x=0;
    people3_pose_rel.orientation.y=0;
    people3_pose_rel.orientation.z=0;

    people4_pose_rel.position.x=(people4_pose.position.x-robot_pose.position.x)*cos(theta1)+(people4_pose.position.y-robot_pose.position.y)*sin(theta1);
    people4_pose_rel.position.y=(robot_pose.position.x-people4_pose.position.x)*sin(theta1)+(people4_pose.position.y-robot_pose.position.y)*cos(theta1);
    people4_pose_rel.position.z=people4_pose.position.z-robot_pose.position.z;
    people4_pose_rel.orientation.w=1;
    people4_pose_rel.orientation.x=0;
    people4_pose_rel.orientation.y=0;
    people4_pose_rel.orientation.z=0;

    people5_pose_rel.position.x=(people5_pose.position.x-robot_pose.position.x)*cos(theta1)+(people5_pose.position.y-robot_pose.position.y)*sin(theta1);
    people5_pose_rel.position.y=(robot_pose.position.x-people5_pose.position.x)*sin(theta1)+(people5_pose.position.y-robot_pose.position.y)*cos(theta1);
    people5_pose_rel.position.z=people5_pose.position.z-robot_pose.position.z;
    people5_pose_rel.orientation.w=1;
    people5_pose_rel.orientation.x=0;
    people5_pose_rel.orientation.y=0;
    people5_pose_rel.orientation.z=0;

}
