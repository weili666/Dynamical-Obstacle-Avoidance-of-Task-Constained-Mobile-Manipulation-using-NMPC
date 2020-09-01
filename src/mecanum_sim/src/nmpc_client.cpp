#include "ros/ros.h"
#include <mecanum_sim/NMPCStep.h>
#include <iostream>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include <std_msgs/Float32.h>
#include <PositionAll.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include <JacobianPoint.h>
#include <time.h>

using namespace std;
//I'm going to implement user-specified twist values later.
const float WHEEL_RAD = 0.18;//0.1016; // meters
const float WHEELBASE = 0.76;//0.466725; // meters
const float TRACK = 0.68;//0.2667; // meters

geometry_msgs::Pose robot_pose;
struct nearPointArm
{
    nearPointArm(int num, double x_, double y_, double z_){
        index_of_arm = num;
        x = x_;
        y = y_;
        z = z_;
    }
    int index_of_arm;
    double x, y, z;
};

void cb(const gazebo_msgs::LinkStatesConstPtr & link_msgs)
{
    robot_pose = link_msgs->pose[1];
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_plan_client");
    ros::NodeHandle n;
    ros::Publisher fl_pub = n.advertise<std_msgs::Float32>("front_left",1,true);
    ros::Publisher fr_pub = n.advertise<std_msgs::Float32>("front_right",1,true);
    ros::Publisher bl_pub = n.advertise<std_msgs::Float32>("back_left",1,true);
    ros::Publisher br_pub = n.advertise<std_msgs::Float32>("back_right",1,true);
    ros::Rate loop_rate(20);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double vx,vy,w;
    vx=0;vy=0;w=0.7853;
    double v_fl=vx+vy-w*(TRACK+WHEELBASE)/2;
    double v_fr=-vx+vy+w*(TRACK+WHEELBASE)/2;
    double v_bl=-vx+vy-w*(TRACK+WHEELBASE)/2;
    double v_br=vx+vy+w*(TRACK+WHEELBASE)/2;
    v_fl=v_fl/WHEEL_RAD;
    v_fr=v_fr/WHEEL_RAD;
    v_bl=v_bl/WHEEL_RAD;
    v_br=v_br/WHEEL_RAD;
    std_msgs::Float32 vfl;
    vfl.data=v_fl;
    std_msgs::Float32 vfr;
    vfr.data=v_fr;
    std_msgs::Float32 vbl;
    vbl.data=v_bl;
    std_msgs::Float32 vbr;
    vbr.data=v_br;
    int c=0;

    KDL::Tree kdl_tree;
    cout<<"hello world!!"<<endl;

    string robot_desc_string;
    n.param("robot_description",robot_desc_string,string());
    if(!kdl_parser::treeFromString(robot_desc_string,kdl_tree))
    {
        cout<<"Failed to construct kdl tree"<<endl;
    }
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver;
    KDL::Chain arm_chain;
    if(!kdl_tree.getChain("base_link", "ee_link", arm_chain))
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
    }
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr = boost::make_shared<KDL::Chain>(arm_chain);
    std::cout << "KDL chain has " << kdl_chain_ptr->getNrOfSegments() << " segments and " << kdl_chain_ptr->getNrOfJoints() << " joints." << std::endl;
    std::cout << "Joints: ";
    for (unsigned int i = 0; i < kdl_chain_ptr->getNrOfSegments(); i++)
        std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
    std::cout << std::endl;

    jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(arm_chain));

    //------------init scene------------//

    planning_scene_monitor::PlanningSceneMonitorPtr scene(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    cout<<"hello world 1"<<endl;
    scene->startStateMonitor();
    scene->startSceneMonitor();
    scene->startWorldGeometryMonitor();

    //------------init robot------------//

    moveit::planning_interface::MoveGroup *robot = new moveit::planning_interface::MoveGroup("manipulator");
    moveit::planning_interface::MoveGroup::Plan plan;
    moveit_msgs::RobotState robot_state;
    cout<<"Print robot state"<<endl;
    robot_state::RobotStatePtr current_state = robot->getCurrentState();
    cout<<current_state<<endl;
    cout<<"hello world 2"<<endl;
    robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
    robot->setNumPlanningAttempts(4);
    robot->setPlannerId("RRTConnectkConfigDefault");
    robot->setStartState(robot_state);
//    std::vector<double> joints;
//    joints.push_back(0);
//    joints.push_back(-2.83);
//    joints.push_back(2.76);
//    joints.push_back(-1.59);
//    joints.push_back(0);
//    joints.push_back(0);
//    robot->setJointValueTarget(joints);
//    bool success = false;
//    success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    cout<<"hello world 3"<<endl;
//    robot->execute(plan);
//    cout<<"hello world 4"<<endl;


//    fl_pub.publish(vfl);
//    fr_pub.publish(vfr);
//    bl_pub.publish(vbl);
//    br_pub.publish(vbr);//move robot
//    //ROS_INFO("BRILLIANT");
//    ros::Duration(2.0).sleep();

//    vfl.data=0;
//    vfr.data=0;
//    vbl.data=0;
//    vbr.data=0;
//    fl_pub.publish(vfl);
//    fr_pub.publish(vfr);
//    bl_pub.publish(vbl);
//    br_pub.publish(vbr);//move robot


    //=====================calculate the move to escape the obstacle======================//
    double x_o, y_o, z_o;
    x_o = -0.5;
    y_o = 0.5;
    z_o = 0.5;
    ros::ServiceClient client = n.serviceClient<mecanum_sim::NMPCStep>("nmpc_plan");
    ros::Publisher speed_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ur10_arm/ur_driver/joint_speed", 1000);
    mecanum_sim::NMPCStep srv;

    nearPointArm np1(1, 0, 0, 0);
    nearPointArm np2(2, 0, 0, 0.176);
    nearPointArm np3(2, -0.1, 0, 0.176);
    nearPointArm np4(2, -0.2, 0, 0.176);
    nearPointArm np5(2, -0.3, 0, 0.176);
    nearPointArm np6(2, -0.4, 0, 0.176);
    nearPointArm np7(2, -0.5, 0, 0.176);
    nearPointArm np8(2, -0.6, 0, 0.176);
    nearPointArm np9(3, 0, 0, 0.048);
    nearPointArm np10(3, -0.1, 0, 0.048);
    nearPointArm np11(3, -0.2, 0, 0.048);
    nearPointArm np12(3, -0.3, 0, 0.048);
    nearPointArm np13(3, -0.4, 0, 0.048);
    nearPointArm np14(3, -0.5, 0, 0.048);
    nearPointArm np15(3, -0.6, 0, 0.048);
    nearPointArm np16(4, 0, 0, 0);
    nearPointArm np17(5, 0, 0, 0);
    nearPointArm np18(6, 0, 0, 0);


bool continue_move = true;
double distance_threshold = 0.7;

while(continue_move)
{
    vector<double> joint_now;
    joint_now = robot->getCurrentJointValues();

    ros::Subscriber sub_joint=n.subscribe("/gazebo/link_states", 10, cb);
    int r=5;
    while(r)
    {
    ros::spinOnce();
    loop_rate.sleep();
    r--;
    }
    double x0, y0, alpha;
    x0 = robot_pose.position.x;
    y0 = robot_pose.position.y;
    alpha = 2*acos(robot_pose.orientation.w);
    cout<<"the pose of robot is: x0:"<<x0<<" ,y0:"<<y0<<", alpha:"<<alpha<<endl;
    vector<JacobianPoint> jps;
    JacobianPoint jp1(np1.index_of_arm, np1.x, np1.y, np1.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp1);
    JacobianPoint jp2(np2.index_of_arm, np2.x, np2.y, np2.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp2);
    JacobianPoint jp3(np3.index_of_arm, np3.x, np3.y, np3.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp3);
    JacobianPoint jp4(np4.index_of_arm, np4.x, np4.y, np4.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp4);
    JacobianPoint jp5(np5.index_of_arm, np5.x, np5.y, np5.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp5);
    JacobianPoint jp6(np6.index_of_arm, np6.x, np6.y, np6.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp6);
    JacobianPoint jp7(np7.index_of_arm, np7.x, np7.y, np7.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp7);
    JacobianPoint jp8(np8.index_of_arm, np8.x, np8.y, np8.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp8);
    JacobianPoint jp9(np9.index_of_arm, np9.x, np9.y, np9.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp9);
    JacobianPoint jp10(np10.index_of_arm, np10.x, np10.y, np10.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp10);
    JacobianPoint jp11(np11.index_of_arm, np11.x, np11.y, np11.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp11);
    JacobianPoint jp12(np12.index_of_arm, np12.x, np12.y, np12.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp12);
    JacobianPoint jp13(np13.index_of_arm, np13.x, np13.y, np13.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp13);
    JacobianPoint jp14(np14.index_of_arm, np14.x, np14.y, np14.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp14);
    JacobianPoint jp15(np15.index_of_arm, np15.x, np15.y, np15.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp15);
    JacobianPoint jp16(np16.index_of_arm, np16.x, np16.y, np16.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp16);
    JacobianPoint jp17(np17.index_of_arm, np17.x, np17.y, np17.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp17);
    JacobianPoint jp18(np18.index_of_arm, np18.x, np18.y, np18.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp18);
    vector<JacobianPoint>::iterator iter;
    double dist = 10000;
    int index = 0;
    int ind = 0;
    for(iter = jps.begin(); iter != jps.end(); iter ++)
    {
        double d = iter->calcDistanceToObstacle(x_o, y_o, z_o);
        cout<<"the distance between the "<<ind<<" th point in the arm and the obstacle is: "<<d<<endl;
        if(d < dist)
        {
            dist = d;
            index = ind;
        }
        ind ++;
    }
    if(dist > distance_threshold)
    {
        continue_move = false;
    }
    cout<<"the nearest point is the "<<index<<"th"<<endl;
    jps[index].calcJacobianWhole();
    cout<<"finish calculate the whole jacobian."<<endl;
    jps[index].calcJacobianNear();
    cout<<"finish calculate the near jacobian."<<endl;
    jps[index].calcJacobianStar();
    cout<<"finish calculate the jacobian star."<<endl;
    srv.request.jacobian.resize(27);
    srv.request.jacobian = {(float)jps[index].JacobianStar.at<double>(0, 0), (float)jps[index].JacobianStar.at<double>(0, 1), (float)jps[index].JacobianStar.at<double>(0, 2),
                           (float)jps[index].JacobianStar.at<double>(0, 3), (float)jps[index].JacobianStar.at<double>(0, 4), (float)jps[index].JacobianStar.at<double>(0, 5),
                           (float)jps[index].JacobianStar.at<double>(0, 6), (float)jps[index].JacobianStar.at<double>(0, 7), (float)jps[index].JacobianStar.at<double>(0, 8),
                           (float)jps[index].JacobianStar.at<double>(1, 0), (float)jps[index].JacobianStar.at<double>(1, 1), (float)jps[index].JacobianStar.at<double>(1, 2),
                           (float)jps[index].JacobianStar.at<double>(1, 3), (float)jps[index].JacobianStar.at<double>(1, 4), (float)jps[index].JacobianStar.at<double>(1, 5),
                           (float)jps[index].JacobianStar.at<double>(1, 6), (float)jps[index].JacobianStar.at<double>(1, 7), (float)jps[index].JacobianStar.at<double>(1, 8),
                           (float)jps[index].JacobianStar.at<double>(2, 0), (float)jps[index].JacobianStar.at<double>(2, 1), (float)jps[index].JacobianStar.at<double>(2, 2),
                           (float)jps[index].JacobianStar.at<double>(2, 3), (float)jps[index].JacobianStar.at<double>(2, 4), (float)jps[index].JacobianStar.at<double>(2, 5),
                           (float)jps[index].JacobianStar.at<double>(2, 6), (float)jps[index].JacobianStar.at<double>(2, 7), (float)jps[index].JacobianStar.at<double>(2, 8)};
    srv.request.xstates.resize(4);
    srv.request.xstates = {(float)jps[index].x1, (float)jps[index].x2, (float)jps[index].x3, (float)jps[index].x4};
    if (client.call(srv))
     {
       ROS_INFO("Success to call service");
     }
     else
     {
       ROS_ERROR("Failed to call service");
       return 1;
     }
    double w1, w2, w3, w4, w5, w6, w7, w8, w9;
    w1 = srv.response.omega1; w2 = srv.response.omega2; w3 = srv.response.omega3; w4 = srv.response.omega4; w5 = srv.response.omega5;
    w6 = srv.response.omega6; w7 = srv.response.omega7; w8 = srv.response.omega8; w9 = srv.response.omega9;
    jps[index].calcAngularVelocities(w1, w2, w3, w4, w5, w6, w7, w8, w9);

    trajectory_msgs::JointTrajectoryPoint trjp;
    trajectory_msgs::JointTrajectory trj;
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trj.points.push_back(trjp);
    ros::Time be = ros::Time::now();
    trj.header.stamp = ros::Time::now();
    cout<<"the real velocity of joint1 :"<<jps[index].dq1;
    cout<<"the real velocity of joint2 :"<<jps[index].dq2;
    cout<<"the real velocity of joint3 :"<<jps[index].dq3;
    cout<<"the real velocity of joint4 :"<<jps[index].dq4;
    cout<<"the real velocity of joint5 :"<<jps[index].dq5;
    cout<<"the real velocity of joint6 :"<<jps[index].dq6;
    cout<<"the real velocity of x :"<<jps[index].dx0;
    cout<<"the real velocity of y :"<<jps[index].dy0;
    cout<<"the real velocity of alpha :"<<jps[index].dph;
    trj.points[0].velocities[0] = jps[index].dq1;
    trj.points[0].velocities[1] = jps[index].dq1;
    trj.points[0].velocities[2] = jps[index].dq1;
    trj.points[0].velocities[3] = jps[index].dq1;
    trj.points[0].velocities[4] = jps[index].dq1;
    trj.points[0].velocities[5] = jps[index].dq1;

    vx=jps[index].dx0*sin(alpha)-jps[index].dy0*cos(alpha);
    vy=jps[index].dx0*cos(alpha)+jps[index].dy0*sin(alpha);
    w=jps[index].dph;
    v_fl=vx+vy-w*(TRACK+WHEELBASE)/2;
    v_fr=-vx+vy+w*(TRACK+WHEELBASE)/2;
    v_bl=-vx+vy-w*(TRACK+WHEELBASE)/2;
    v_br=vx+vy+w*(TRACK+WHEELBASE)/2;
    v_fl=v_fl/WHEEL_RAD;
    v_fr=v_fr/WHEEL_RAD;
    v_bl=v_bl/WHEEL_RAD;
    v_br=v_br/WHEEL_RAD;
    vfl.data=v_fl;
    vfr.data=v_fr;
    vbl.data=v_bl;
    vbr.data=v_br;

    speed_pub.publish(trj);
    fl_pub.publish(vfl);
    fr_pub.publish(vfr);
    bl_pub.publish(vbl);
    br_pub.publish(vbr);

    ros::Duration(1.0).sleep();

    trj.points[0].velocities[0] = 0.0;
    trj.points[0].velocities[1] = 0.0;
    trj.points[0].velocities[2] = 0.0;
    trj.points[0].velocities[3] = 0.0;
    trj.points[0].velocities[4] = 0.0;
    trj.points[0].velocities[5] = 0.0;
    vfl.data=0;
    vfr.data=0;
    vbl.data=0;
    vbr.data=0;

    speed_pub.publish(trj);
    fl_pub.publish(vfl);
    fr_pub.publish(vfr);
    bl_pub.publish(vbl);
    br_pub.publish(vbr);

}


}
