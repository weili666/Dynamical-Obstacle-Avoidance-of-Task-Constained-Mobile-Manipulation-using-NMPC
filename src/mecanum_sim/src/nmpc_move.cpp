#include "ros/ros.h"
#include <iostream>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <mecanum_sim/NMPCStep.h>
#include <iostream>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include <std_msgs/Float32.h>
#include <PositionAll.h>
#include <eigen_conversions/eigen_msg.h>

#include <stdio.h>
#include <stdlib.h>
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

#include <thread>

#include <JacobianPoint.h>
#include <time.h>

//#define WITH_NMPC

//#define FROM_BACK

using namespace std;
USING_NAMESPACE_ACADO

//I'm going to implement user-specified twist values later.
const float WHEEL_RAD = 0.1254;
const float WHEELBASE = 0.6940;
const float TRACK = 0.4765;

geometry_msgs::Pose robot_pose;
geometry_msgs::Pose obstacle_pose;
geometry_msgs::Pose end_pose;

double yita = 0.1;

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
#ifdef FROM_BACK
    obstacle_pose = link_msgs->pose[22];//13 is person_walking, 22 is person_walking_0
#else
    obstacle_pose = link_msgs->pose[13];//13 is person_walking, 22 is person_walking_0
#endif
    end_pose = link_msgs->pose[7];
}

class RobotBaseControl
{
public:
    RobotBaseControl(ros::NodeHandle nh_,double x0_, double y0_, double alpha_, double x0_1_, double y0_1_, double alpha_1_, double vx_, double vy_, double w_){
        nh = nh_;
        x0_1 = x0_1_;
        y0_1 = y0_1_;
        alpha_1 = alpha_1_;
        x0 = x0_;
        y0 = y0_;
        alpha = alpha_;
        vx_base = vx_;
        vy_base = vy_;
        w_base = w_;
    }

    ~RobotBaseControl(){}
    void cb2(const gazebo_msgs::LinkStatesConstPtr & link_msgs);
    void control_base()
    {
        base_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
        sub_base = nh.subscribe("/gazebo/link_states", 1, &RobotBaseControl::cb2, this);
        ros::Rate loop_rate(20);
        int rl=20;
        while(rl)
        {
        ros::spinOnce();
        loop_rate.sleep();
        rl--;
        }
    }
protected:
    ros::NodeHandle nh;
    geometry_msgs::Twist vw;
    double x0_1, y0_1, alpha_1;
    double x0, y0, alpha;
    ros::Publisher base_pub;
    ros::Subscriber sub_base;
    double vx_base, vy_base, w_base;

};

void RobotBaseControl::cb2(const gazebo_msgs::LinkStatesConstPtr & link_msgs)
{
    geometry_msgs::Pose robot_pose_temp = link_msgs->pose[1];
    double x1, y1, alpha1;
    x1 = robot_pose_temp.position.x;
    y1 = robot_pose_temp.position.y;
    alpha1 = 2*acos(robot_pose_temp.orientation.w);

    double delta_x_rel = (x0_1-x0)*cos(alpha)+(y0_1-y0)*sin(alpha);
    double delta_y_rel = -(x0_1-x0)*sin(alpha)+(y0_1-y0)*cos(alpha);
    double delta_x_rel_temp = (x1-x0)*cos(alpha)+(y1-y0)*sin(alpha);
    double delta_y_rel_temp = -(x1-x0)*sin(alpha)+(y1-y0)*cos(alpha);

    if(abs(delta_x_rel_temp)<abs(delta_x_rel))
    {
        vw.linear.x = 2*yita*vx_base;
    }
    else
    {
        vw.linear.x = 0;
    }
    if(abs(delta_y_rel_temp)<abs(delta_y_rel))
    {
        vw.linear.y = 2*yita*vy_base;
    }
    else
    {
        vw.linear.y = 0;
    }
    if(abs(alpha1-alpha)<abs(alpha_1-alpha))
    {
        vw.angular.z = 0.35*yita*w_base*180/M_PI;
    }
    else
    {
        using namespace cv;
        vw.angular.z = 0;
    }


    base_pub.publish(vw);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"robot_mover_mark_two");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);

    double vx,vy,w;


    KDL::Tree kdl_tree;
    cout<<"hello world!!"<<endl;

    string robot_desc_string;
    n.param("robot_description",robot_desc_string,string());
    if(!kdl_parser::treeFromString(robot_desc_string,kdl_tree))
    {
        cout<<"Failed to construct kdl tree"<<endl;
    }
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver;
    boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver;
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
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(arm_chain));



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

    //=====================calculate the move to escape the obstacle======================//
    double x_o, y_o, z_o;


    nearPointArm np1(1, 0, 0, 0);
    nearPointArm np2(2, 0, 0, 0.176);
    nearPointArm np3(2, -0.1, 0, 0.176);
    nearPointArm np4(2, -0.2, 0, 0.176);
    nearPointArm np5(2, -0.3, 0, 0.176);
    nearPointArm np6(2, -0.4, 0, 0.176);
    nearPointArm np7(2, -0.5, 0, 0.176);
    nearPointArm np8(2, -0.612, 0, 0.176);
    nearPointArm np9(3, 0, 0, 0.048);
    nearPointArm np10(3, -0.1, 0, 0.048);
    nearPointArm np11(3, -0.2, 0, 0.048);
    nearPointArm np12(3, -0.3, 0, 0.048);
    nearPointArm np13(3, -0.4, 0, 0.048);
    nearPointArm np14(3, -0.5, 0, 0.048);
    nearPointArm np15(3, -0.572, 0, 0.048);
    nearPointArm np16(4, 0, 0, 0);
    nearPointArm np17(5, 0, 0, 0);
    nearPointArm np18(0, 0.42, 0.26, 0.3784);
    nearPointArm np19(0, 0.42, -0.26, 0.3784);
    nearPointArm np20(0, -0.42, 0.26, 0.3784);
    nearPointArm np21(0, -0.42, -0.26, 0.3784);
    nearPointArm np22(0, 0.42, 0, 0.3784);
    nearPointArm np23(0, 0, 0.26, 0.3784);
    nearPointArm np24(0, -0.42, 0, 0.3784);
    nearPointArm np25(0, 0, -0.26, 0.3784);
    nearPointArm np26(0, 0.42, 0.38, 0.15);
    nearPointArm np27(0, -0.42, 0.38, 0.15);
    nearPointArm np28(0, -0.42, -0.38, 0.15);
    nearPointArm np29(0, 0.42, -0.38, 0.15);


    double x0, y0, alpha;
    double xe, ye, alphae;
    double x0_1, y0_1, alpha_1;

    bool continue_move = true;
    double distance_threshold = 2.0;
    double velocity = 0.7;

while(continue_move)
{

    vector<double> joint_now;

    joint_now = robot->getCurrentJointValues();

    ros::Subscriber sub_joint=n.subscribe("/gazebo/link_states", 10, cb);
    int rl=5;
    while(rl)
    {
    ros::spinOnce();
    loop_rate.sleep();
    rl--;
    }


    //=====================obstacle move step=======================//

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::ModelState modelstate;


#ifdef FROM_BACK
    modelstate.model_name = "person_walking_0";
    modelstate.reference_frame = "world";
    x_o = obstacle_pose.position.x;//-1.0;
    y_o = obstacle_pose.position.y+0.2;//0.5;
    z_o = 0.5;//1.5;
#else
    modelstate.model_name = "person_walking";
    modelstate.reference_frame = "world";
    x_o = obstacle_pose.position.x+0.2;//-1.0;
    y_o = obstacle_pose.position.y;//0.5;
    z_o = 1.0;//0.5;//1.5;
#endif

    cout<<"the obstacle x :"<<x_o<<", y_o :"<<y_o<<", z_o :"<<z_o<<endl;

    //==============================================================//

    x0 = robot_pose.position.x;
    y0 = robot_pose.position.y;
    alpha = 2*acos(robot_pose.orientation.w);
    xe = end_pose.position.x;
    ye = end_pose.position.y;
    alphae = 2*acos(end_pose.orientation.w);

    cout<<"the pose of robot is: x0:"<<x0<<" ,y0:"<<y0<<", alpha:"<<alpha<<endl;
    cout<<"the pose of robot end is: xe:"<<xe<<" ,ye:"<<ye<<", alphae:"<<alphae<<endl;
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
    JacobianPoint jp19(np19.index_of_arm, np19.x, np19.y, np19.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp19);
    JacobianPoint jp20(np20.index_of_arm, np20.x, np20.y, np20.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp20);
    JacobianPoint jp21(np21.index_of_arm, np21.x, np21.y, np21.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp21);
    JacobianPoint jp22(np22.index_of_arm, np22.x, np22.y, np22.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp22);
    JacobianPoint jp23(np23.index_of_arm, np23.x, np23.y, np23.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp23);
    JacobianPoint jp24(np24.index_of_arm, np24.x, np24.y, np24.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp24);
    JacobianPoint jp25(np25.index_of_arm, np25.x, np25.y, np25.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp25);
    JacobianPoint jp26(np26.index_of_arm, np26.x, np26.y, np26.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp26);
    JacobianPoint jp27(np27.index_of_arm, np27.x, np27.y, np27.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp27);
    JacobianPoint jp28(np28.index_of_arm, np28.x, np28.y, np28.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp28);
    JacobianPoint jp29(np29.index_of_arm, np29.x, np29.y, np29.z, x0, y0, alpha, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]);
    jps.push_back(jp29);
    vector<JacobianPoint>::iterator iter;
    double dist2 = -10000;//max of 3
    double dist2_ = 10000;//min of 3
    double min_dist = 10000;
    double dists[3] = {0, 0, 0};
    int index = 0;
    int index2 = 0;//max index of 3
    int index2_ = 0;//min index of 3
    int indexes[3] = {0, 0, 0};
    int ind = 3;
    int i = 0;
    for(iter = jps.begin(); iter!= jps.begin()+3; iter++)
    {
        double d = iter->calcDistanceToObstacle(x_o, y_o, z_o);
        dists[i] = d; i++;
        indexes[i] = i;
    }
    for(iter = jps.begin()+3; iter != jps.end(); iter ++)
    {
        dist2 = -10000;//max of 3
        dist2_ = 10000;//min of 3
        index2 = 0;//max index of 3
        index2_ = 0;//min index of 3
        double d = iter->calcDistanceToObstacle(x_o, y_o, z_o);
        cout<<"the distance between the "<<ind<<" th point in the arm and the obstacle is: "<<d<<endl;
        for(int j = 0; j < 3; j ++)
        {
            if(dists[j] > dist2)
            {
                dist2 = dists[j];
                index2 = j;
            }
            else if(dists[j] < dist2_)
            {
                dist2_ = dists[j];
                index2_ = j;
            }
        }
        if(dist2_ < min_dist)
        {
            min_dist = dist2_;
        }
        if(d < dist2)
        {
            dists[index2] = d;
            indexes[index2] = ind;
        }
        ind ++;
    }
    if(min_dist > distance_threshold)
    {
        continue_move = false;
        cout<<"<<<<<<<...don't need to continue...>>>>>>>"<<endl;
    }


    //=========================================end clarify================================================//

    cout<<"the nearest point is the "<<indexes[index2_]<<"th"<<endl;
    vector<double> j11, j12, j13, j14, j15, j16, j17, j18, j19;
    vector<double> j21, j22, j23, j24, j25, j26, j27, j28, j29;
    vector<double> j31, j32, j33, j34, j35, j36, j37, j38, j39;
    vector<double> xstates;
    for(int i = 0; i < 3; i ++)
    {

    index = indexes[i];
    jps[index].calcJacobianWhole();
    jps[index].calcJacobianNear();
    cout<<"finish calculate the near jacobian."<<endl;
    jps[index].calcJacobianStar();
    cout<<"index :"<<index<<endl;
    double jacobian[]= {jps[index].JacobianStar.at<double>(0, 0), jps[index].JacobianStar.at<double>(0, 1), jps[index].JacobianStar.at<double>(0, 2),
               jps[index].JacobianStar.at<double>(0, 3), jps[index].JacobianStar.at<double>(0, 4), jps[index].JacobianStar.at<double>(0, 5),
               jps[index].JacobianStar.at<double>(0, 6), jps[index].JacobianStar.at<double>(0, 7), jps[index].JacobianStar.at<double>(0, 8),
               jps[index].JacobianStar.at<double>(1, 0), jps[index].JacobianStar.at<double>(1, 1), jps[index].JacobianStar.at<double>(1, 2),
               jps[index].JacobianStar.at<double>(1, 3), jps[index].JacobianStar.at<double>(1, 4), jps[index].JacobianStar.at<double>(1, 5),
               jps[index].JacobianStar.at<double>(1, 6), jps[index].JacobianStar.at<double>(1, 7), jps[index].JacobianStar.at<double>(1, 8),
               jps[index].JacobianStar.at<double>(2, 0), jps[index].JacobianStar.at<double>(2, 1), jps[index].JacobianStar.at<double>(2, 2),
               jps[index].JacobianStar.at<double>(2, 3), jps[index].JacobianStar.at<double>(2, 4), jps[index].JacobianStar.at<double>(2, 5),
               jps[index].JacobianStar.at<double>(2, 6), jps[index].JacobianStar.at<double>(2, 7), jps[index].JacobianStar.at<double>(2, 8)};
    xstates.push_back(jps[index].x1); xstates.push_back(jps[index].x2); xstates.push_back(jps[index].x3); xstates.push_back(jps[index].x4);

    cout<<"x1:"<<jps[index].x1<<", x2:"<<jps[index].x2<<", x3:"<<jps[index].x3<<", x4:"<<jps[index].x4<<endl;
    j11.push_back(jacobian[0]); j12.push_back(jacobian[1]); j13.push_back(jacobian[2]); j14.push_back(jacobian[3]); j15.push_back(jacobian[4]); j16.push_back(jacobian[5]); j17.push_back(jacobian[6]); j18.push_back(jacobian[7]); j19.push_back(jacobian[8]);
    j21.push_back(jacobian[9]); j22.push_back(jacobian[10]); j23.push_back(jacobian[11]); j24.push_back(jacobian[12]); j25.push_back(jacobian[13]); j26.push_back(jacobian[14]); j27.push_back(jacobian[15]); j28.push_back(jacobian[16]); j29.push_back(jacobian[17]);
    j31.push_back(jacobian[18]); j32.push_back(jacobian[19]); j33.push_back(jacobian[20]); j34.push_back(jacobian[21]); j35.push_back(jacobian[22]); j36.push_back(jacobian[23]); j37.push_back(jacobian[24]); j38.push_back(jacobian[25]); j39.push_back(jacobian[26]);

    cout<<"j11:"<<jacobian[0]<<",j12:"<<jacobian[1]<<",j13:"<<jacobian[2]<<",j14:"<<jacobian[3]<<",j15:"<<jacobian[4]<<",j16:"<<jacobian[5]<<",j17:"<<jacobian[6]<<",j18:"<<jacobian[7]<<",j19:"<<jacobian[8]<<endl;
        cout<<"j21:"<<jacobian[9]<<",j22:"<<jacobian[10]<<",j23:"<<jacobian[11]<<",24:"<<jacobian[12]<<",j25:"<<jacobian[13]<<",j16:"<<jacobian[14]<<",j27:"<<jacobian[15]<<",j28:"<<jacobian[16]<<",j29:"<<jacobian[17]<<endl;
            cout<<"j31:"<<jacobian[18]<<",j32:"<<jacobian[19]<<",j33:"<<jacobian[20]<<",j34:"<<jacobian[21]<<",j35:"<<jacobian[22]<<",j36:"<<jacobian[23]<<",j37:"<<jacobian[24]<<",j38:"<<jacobian[25]<<",j39:"<<jacobian[26]<<endl;
    }

    double distance_ = 10000;

    for(int i = 0; i < 3; i ++)
    {
        cout<<"the smallest 3 distances is :"<<dists[i]<<", index is:"<<indexes[i]<<endl;
        if(dists[i] < distance_)
        {
            distance_ = dists[i];
            index = indexes[i];
        }
    }
    cout<<"the smallest 1 distances is :"<<distance_<<", index is:"<<index<<endl;



#ifdef WITH_NMPC
    //==================NMPC===================//


    DifferentialState x1;
    DifferentialState x2;
    DifferentialState x3;
    DifferentialState x4;
    DifferentialState x11;
    DifferentialState x12;
    DifferentialState x13;
    DifferentialState x14;
    DifferentialState x21;
    DifferentialState x22;
    DifferentialState x23;
    DifferentialState x24;


    Control w1;
    Control w2;
    Control w3;
    Control w4;
    Control w5;
    Control w6;
    Control w7;
    Control w8;
    Control w9;


    Parameter T;
    DifferentialEquation f;


    f << dot(x1) == -x1*x1*((x2*j11[0]+x3*j21[0]+x4*j31[0])*w1+(x2*j12[0]+x3*j22[0]+x4*j32[0])*w2+(x2*j13[0]+x3*j23[0]+x4*j33[0])*w3+(x2*j14[0]+x3*j24[0]+x4*j34[0])*w4+(x2*j15[0]+x3*j25[0]+x4*j35[0])*w5+(x2*j16[0]+x3*j26[0]+x4*j36[0])*w6+(x2*j17[0]+x3*j27[0]+x4*j37[0])*w7+(x2*j18[0]+x3*j28[0]+x4*j38[0])*w8+(x2*j19[0]+x3*j29[0]+x4*j39[0])*w9);
    f << dot(x2) == x1*(j11[0]*w1+j12[0]*w2+j13[0]*w3+j14[0]*w4+j15[0]*w5+j16[0]*w6+j17[0]*w7+j18[0]*w8+j19[0]*w9)-x1*x2*((x2*j11[0]+x3*j21[0]+x4*j31[0])*w1+(x2*j12[0]+x3*j22[0]+x4*j32[0])*w2+(x2*j13[0]+x3*j23[0]+x4*j33[0])*w3+(x2*j14[0]+x3*j24[0]+x4*j34[0])*w4+(x2*j15[0]+x3*j25[0]+x4*j35[0])*w5+(x2*j16[0]+x3*j26[0]+x4*j36[0])*w6+(x2*j17[0]+x3*j27[0]+x4*j37[0])*w7+(x2*j18[0]+x3*j28[0]+x4*j38[0])*w8+(x2*j19[0]+x3*j29[0]+x4*j39[0])*w9);
    f << dot(x3) == x1*(j21[0]*w1+j22[0]*w2+j23[0]*w3+j24[0]*w4+j25[0]*w5+j26[0]*w6+j27[0]*w7+j28[0]*w8+j29[0]*w9)-x1*x3*((x2*j11[0]+x3*j21[0]+x4*j31[0])*w1+(x2*j12[0]+x3*j22[0]+x4*j32[0])*w2+(x2*j13[0]+x3*j23[0]+x4*j33[0])*w3+(x2*j14[0]+x3*j24[0]+x4*j34[0])*w4+(x2*j15[0]+x3*j25[0]+x4*j35[0])*w5+(x2*j16[0]+x3*j26[0]+x4*j36[0])*w6+(x2*j17[0]+x3*j27[0]+x4*j37[0])*w7+(x2*j18[0]+x3*j28[0]+x4*j38[0])*w8+(x2*j19[0]+x3*j29[0]+x4*j39[0])*w9);
    f << dot(x4) == x1*(j31[0]*w1+j32[0]*w2+j33[0]*w3+j34[0]*w4+j35[0]*w5+j36[0]*w6+j37[0]*w7+j38[0]*w8+j39[0]*w9)-x1*x4*((x2*j11[0]+x3*j21[0]+x4*j31[0])*w1+(x2*j12[0]+x3*j22[0]+x4*j32[0])*w2+(x2*j13[0]+x3*j23[0]+x4*j33[0])*w3+(x2*j14[0]+x3*j24[0]+x4*j34[0])*w4+(x2*j15[0]+x3*j25[0]+x4*j35[0])*w5+(x2*j16[0]+x3*j26[0]+x4*j36[0])*w6+(x2*j17[0]+x3*j27[0]+x4*j37[0])*w7+(x2*j18[0]+x3*j28[0]+x4*j38[0])*w8+(x2*j19[0]+x3*j29[0]+x4*j39[0])*w9);

    f << dot(x11) == -x11*x11*((x12*j11[1]+x13*j21[1]+x14*j31[1])*w1+(x12*j12[1]+x13*j22[1]+x14*j32[1])*w2+(x12*j13[1]+x13*j23[1]+x14*j33[1])*w3+(x12*j14[1]+x13*j24[1]+x14*j34[1])*w4+(x12*j15[1]+x13*j25[1]+x14*j35[1])*w5+(x12*j16[1]+x13*j26[1]+x14*j36[1])*w6+(x12*j17[1]+x13*j27[1]+x14*j37[1])*w7+(x12*j18[1]+x13*j28[1]+x14*j38[1])*w8+(x12*j19[1]+x13*j29[1]+x14*j39[1])*w9);
    f << dot(x12) == x11*(j11[1]*w1+j12[1]*w2+j13[1]*w3+j14[1]*w4+j15[1]*w5+j16[1]*w6+j17[1]*w7+j18[1]*w8+j19[1]*w9)-x11*x12*((x12*j11[1]+x13*j21[1]+x14*j31[1])*w1+(x12*j12[1]+x13*j22[1]+x14*j32[1])*w2+(x12*j13[1]+x13*j23[1]+x14*j33[1])*w3+(x12*j14[1]+x13*j24[1]+x14*j34[1])*w4+(x12*j15[1]+x13*j25[1]+x14*j35[1])*w5+(x12*j16[1]+x13*j26[1]+x14*j36[1])*w6+(x12*j17[1]+x13*j27[1]+x14*j37[1])*w7+(x12*j18[1]+x13*j28[1]+x14*j38[1])*w8+(x12*j19[1]+x13*j29[1]+x14*j39[1])*w9);
    f << dot(x13) == x11*(j21[1]*w1+j22[1]*w2+j23[1]*w3+j24[1]*w4+j25[1]*w5+j26[1]*w6+j27[1]*w7+j28[1]*w8+j29[1]*w9)-x11*x13*((x12*j11[1]+x13*j21[1]+x14*j31[1])*w1+(x12*j12[1]+x13*j22[1]+x14*j32[1])*w2+(x12*j13[1]+x13*j23[1]+x14*j33[1])*w3+(x12*j14[1]+x13*j24[1]+x14*j34[1])*w4+(x12*j15[1]+x13*j25[1]+x14*j35[1])*w5+(x12*j16[1]+x13*j26[1]+x14*j36[1])*w6+(x12*j17[1]+x13*j27[1]+x14*j37[1])*w7+(x12*j18[1]+x13*j28[1]+x14*j38[1])*w8+(x12*j19[1]+x13*j29[1]+x14*j39[1])*w9);
    f << dot(x14) == x11*(j31[1]*w1+j32[1]*w2+j33[1]*w3+j34[1]*w4+j35[1]*w5+j36[1]*w6+j37[1]*w7+j38[1]*w8+j39[1]*w9)-x11*x14*((x12*j11[1]+x13*j21[1]+x14*j31[1])*w1+(x12*j12[1]+x13*j22[1]+x14*j32[1])*w2+(x12*j13[1]+x13*j23[1]+x14*j33[1])*w3+(x12*j14[1]+x13*j24[1]+x14*j34[1])*w4+(x12*j15[1]+x13*j25[1]+x14*j35[1])*w5+(x12*j16[1]+x13*j26[1]+x14*j36[1])*w6+(x12*j17[1]+x13*j27[1]+x14*j37[1])*w7+(x12*j18[1]+x13*j28[1]+x14*j38[1])*w8+(x12*j19[1]+x13*j29[1]+x14*j39[1])*w9);

    f << dot(x21) == -x21*x21*((x22*j11[2]+x23*j21[2]+x24*j31[2])*w1+(x22*j12[2]+x23*j22[2]+x24*j32[2])*w2+(x22*j13[2]+x23*j23[2]+x24*j33[2])*w3+(x22*j14[2]+x23*j24[2]+x24*j34[2])*w4+(x22*j15[2]+x23*j25[2]+x24*j35[2])*w5+(x22*j16[2]+x13*j26[2]+x24*j36[2])*w6+(x22*j17[2]+x23*j27[2]+x24*j37[2])*w7+(x22*j18[2]+x23*j28[2]+x24*j38[2])*w8+(x22*j19[2]+x23*j29[2]+x24*j39[2])*w9);
    f << dot(x22) == x21*(j11[2]*w1+j12[2]*w2+j13[2]*w3+j14[2]*w4+j15[2]*w5+j16[2]*w6+j17[2]*w7+j18[2]*w8+j19[2]*w9)-x21*x12*((x22*j11[2]+x23*j21[2]+x24*j31[2])*w1+(x22*j12[2]+x23*j22[2]+x24*j32[2])*w2+(x22*j13[2]+x23*j23[2]+x24*j33[2])*w3+(x22*j14[2]+x23*j24[2]+x24*j34[2])*w4+(x22*j15[2]+x23*j25[2]+x24*j35[2])*w5+(x22*j16[2]+x23*j26[2]+x24*j36[2])*w6+(x22*j17[2]+x23*j27[2]+x24*j37[2])*w7+(x22*j18[2]+x23*j28[2]+x24*j38[2])*w8+(x22*j19[2]+x23*j29[2]+x24*j39[2])*w9);
    f << dot(x23) == x21*(j21[2]*w1+j22[2]*w2+j23[2]*w3+j24[2]*w4+j25[2]*w5+j26[2]*w6+j27[2]*w7+j28[2]*w8+j29[2]*w9)-x21*x13*((x22*j11[2]+x23*j21[2]+x24*j31[2])*w1+(x22*j12[2]+x23*j22[2]+x24*j32[2])*w2+(x22*j13[2]+x23*j23[2]+x24*j33[2])*w3+(x22*j14[2]+x23*j24[2]+x24*j34[2])*w4+(x22*j15[2]+x23*j25[2]+x24*j35[2])*w5+(x22*j16[2]+x23*j26[2]+x24*j36[2])*w6+(x22*j17[2]+x23*j27[2]+x24*j37[2])*w7+(x22*j18[2]+x23*j28[2]+x24*j38[2])*w8+(x22*j19[2]+x23*j29[2]+x24*j39[2])*w9);
    f << dot(x24) == x21*(j31[2]*w1+j32[2]*w2+j33[2]*w3+j34[2]*w4+j35[2]*w5+j36[2]*w6+j37[2]*w7+j38[2]*w8+j39[2]*w9)-x21*x14*((x22*j11[2]+x23*j21[2]+x24*j31[2])*w1+(x22*j12[2]+x23*j22[2]+x24*j32[2])*w2+(x22*j13[2]+x23*j23[2]+x24*j33[2])*w3+(x22*j14[2]+x23*j24[2]+x24*j34[2])*w4+(x22*j15[2]+x23*j25[2]+x24*j35[2])*w5+(x22*j16[2]+x23*j26[2]+x24*j36[2])*w6+(x22*j17[2]+x23*j27[2]+x24*j37[2])*w7+(x22*j18[2]+x23*j28[2]+x24*j38[2])*w8+(x22*j19[2]+x23*j29[2]+x24*j39[2])*w9);

    Function h;

    h << x1;
    h << x2;
    h << x3;
    h << x4;
    h << x11;
    h << x12;
    h << x13;
    h << x14;
    h << x21;
    h << x22;
    h << x23;
    h << x24;
    h << w1;
    h << w2;
    h << w3;
    h << w4;
    h << w5;
    h << w6;
    h << w7;
    h << w8;
    h << w9;


    DMatrix Q(21, 21);
//     DMatrix Q(12, 12);
    Q.setIdentity();
    Q(0, 0) = 1.0;
    Q(4, 4) = 1.0;
    Q(8, 8) = 1.0;
    Q(12, 12) = 2.0;
    Q(13, 13) = 2.0;
    Q(14, 14) = 2.0;
    Q(18, 18) = 2.0;
    Q(19, 19) = 2.0;
    Q(20, 20) = 2.0;



    DVector r(21);
//     DVector r(12);
    r.setAll(0.0);


    const double t_start = 0.0;
    const double t_end = 1.0;

    OCP ocp( t_start, t_end, 10 );

    ocp.minimizeLSQ( Q, h, r );

    ocp.subjectTo( f );

    ocp.subjectTo( -0.5 <= w1 <=  0.5   );
    ocp.subjectTo( -0.5 <= w2 <=  0.5   );
    ocp.subjectTo( -0.5 <= w3 <=  0.5   );
    ocp.subjectTo( -0.5 <= w4 <=  0.5   );
    ocp.subjectTo( -0.5 <= w5 <=  0.5   );
    ocp.subjectTo( -0.5 <= w6 <=  0.5   );
    ocp.subjectTo( -0.5 <= w7 <=  0.5   );
    ocp.subjectTo( -0.2 <= w8 <=  0.2   );
    ocp.subjectTo( -0.2 <= w9 <=  0.2   );


//    ocp.subjectTo( AT_START, x1 ==  xstates[0] );
//    ocp.subjectTo( AT_START, x2 ==  xstates[1] );
//    ocp.subjectTo( AT_START, x3 ==  xstates[2] );
//    ocp.subjectTo( AT_START, x4 ==  xstates[3] );
//    ocp.subjectTo( AT_START, x11 ==  xstates[4] );
//    ocp.subjectTo( AT_START, x12 ==  xstates[5] );
//    ocp.subjectTo( AT_START, x13 ==  xstates[6] );
//    ocp.subjectTo( AT_START, x14 ==  xstates[7] );
//    ocp.subjectTo( AT_START, x21 ==  xstates[8] );
//    ocp.subjectTo( AT_START, x22 ==  xstates[9] );
//    ocp.subjectTo( AT_START, x23 ==  xstates[10] );
//    ocp.subjectTo( AT_START, x24 ==  xstates[11] );
    // SETTING UP THE (SIMULATED) PROCESS:
       // -----------------------------------
    OutputFcn identity;

    DynamicSystem dynamicSystem( f,identity );

    Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg( ocp,0.1 );
//    cout<<"haha 1"<<endl;
//    OptimizationAlgorithm alg(ocp);
//    cout<<"haha 2"<<endl;
//    alg.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
//    cout<<"haha 3"<<endl;
//    alg.set( MAX_NUM_ITERATIONS, 3 );
//    cout<<"haha 4"<<endl;
//    alg.set( KKT_TOLERANCE, 1e-10 );
//    cout<<"haha 5"<<endl;
//    alg.solve();
//    cout<<"haha 6"<<endl;

    StaticReferenceTrajectory zeroReference;

    Controller controller( alg,zeroReference );



//    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
//       // ----------------------------------------------------------
    SimulationEnvironment sim( 0.0,1.0,process,controller );

    DVector x_0(12);
    x_0(0) = xstates[0];
    x_0(1) = xstates[1];
    x_0(2) = xstates[2];
    x_0(3) = xstates[3];
    x_0(4) = xstates[4];
    x_0(5) = xstates[5];
    x_0(6) = xstates[6];
    x_0(7) = xstates[7];
    x_0(8) = xstates[8];
    x_0(9) = xstates[9];
    x_0(10) = xstates[10];
    x_0(11) = xstates[11];

    cout<<"x_0 's dim :"<<x_0.getDim()<<", rows:"<<x_0.rows()<<", cols:"<<x_0.cols()<<", is empty:"<<x_0.isEmpty()<<endl;
    sim.init( x_0 );//loop core dumped point

    sim.run( );


    // ...AND PLOT THE RESULTS
          // ----------------------------------------------------------
    VariablesGrid sampledProcessOutput;
    sim.getSampledProcessOutput( sampledProcessOutput );

    VariablesGrid feedbackControl;
//    cout<<"haha 7"<<endl;
//    alg.getControls(feedbackControl);
//    cout<<"haha 8"<<endl;
    sim.getFeedbackControl( feedbackControl );

    sampledProcessOutput.print("/home/weili/catkin_ws/sampledProcessOutput.txt");
    cout<<"rows:"<<sampledProcessOutput.getNumRows()<<", columns:"<<sampledProcessOutput.getNumCols()<<endl;
    cout<<"dim:"<<sampledProcessOutput.getDim()<<endl;
    cout<<"matrix:"<<sampledProcessOutput.getMatrix(1)<<endl;

    feedbackControl.print("/home/weili/catkin_ws/feedbackControl.txt");
    cout<<"rows:"<<feedbackControl.getNumRows()<<", columns:"<<feedbackControl.getNumCols()<<endl;
    cout<<"dim:"<<feedbackControl.getDim()<<endl;
    cout<<"matrix:"<<feedbackControl.getMatrix(1)<<endl;
    DMatrix mc = feedbackControl.getMatrix(1);
    cout<<"the control rules of null-space are:"<<mc(0,0)<<","<<mc(1,0)<<","<<mc(2,0)<<","<<mc(3,0)<<","<<mc(4,0)<<","<<mc(5,0)<<","<<mc(6,0)<<","<<mc(7,0)<<","<<mc(8,0)<<endl;

    //=============================================//
#else
    //==================electrical potential field=====================//

    double k1 = 0.4;
    double k2 = 0.4;
    double k3 = 0.4;
    double w1, w2, w3, w4, w5, w6, w7, w8, w9;
    w1 = k1*xstates[0]*xstates[0]*(xstates[1]*j11[0]+xstates[2]*j21[0]+xstates[3]*j31[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j11[1]+xstates[6]*j21[1]+xstates[7]*j31[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j11[2]+xstates[10]*j21[2]+xstates[11]*j31[2]);
    w2 = k1*xstates[0]*xstates[0]*(xstates[1]*j12[0]+xstates[2]*j22[0]+xstates[3]*j32[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j12[1]+xstates[6]*j22[1]+xstates[7]*j32[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j12[2]+xstates[10]*j22[2]+xstates[11]*j32[2]);
    w3 = k1*xstates[0]*xstates[0]*(xstates[1]*j13[0]+xstates[2]*j23[0]+xstates[3]*j33[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j13[1]+xstates[6]*j23[1]+xstates[7]*j33[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j13[2]+xstates[10]*j23[2]+xstates[11]*j33[2]);
    w4 = k1*xstates[0]*xstates[0]*(xstates[1]*j14[0]+xstates[2]*j24[0]+xstates[3]*j34[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j14[1]+xstates[6]*j24[1]+xstates[7]*j34[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j14[2]+xstates[10]*j24[2]+xstates[11]*j34[2]);
    w5 = k1*xstates[0]*xstates[0]*(xstates[1]*j15[0]+xstates[2]*j25[0]+xstates[3]*j35[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j15[1]+xstates[6]*j25[1]+xstates[7]*j35[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j15[2]+xstates[10]*j25[2]+xstates[11]*j35[2]);
    w6 = k1*xstates[0]*xstates[0]*(xstates[1]*j16[0]+xstates[2]*j26[0]+xstates[3]*j36[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j16[1]+xstates[6]*j26[1]+xstates[7]*j36[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j16[2]+xstates[10]*j26[2]+xstates[11]*j36[2]);
    w7 = k1*xstates[0]*xstates[0]*(xstates[1]*j17[0]+xstates[2]*j27[0]+xstates[3]*j37[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j17[1]+xstates[6]*j27[1]+xstates[7]*j37[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j17[2]+xstates[10]*j27[2]+xstates[11]*j37[2]);
    w8= k1*xstates[0]*xstates[0]*(xstates[1]*j18[0]+xstates[2]*j28[0]+xstates[3]*j38[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j18[1]+xstates[6]*j28[1]+xstates[7]*j38[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j18[2]+xstates[10]*j28[2]+xstates[11]*j38[2]);
    w9 = k1*xstates[0]*xstates[0]*(xstates[1]*j19[0]+xstates[2]*j29[0]+xstates[3]*j39[0])+
         k2*xstates[4]*xstates[4]*(xstates[5]*j19[1]+xstates[6]*j29[1]+xstates[7]*j39[1])+
         k3*xstates[8]*xstates[8]*(xstates[9]*j19[2]+xstates[10]*j29[2]+xstates[11]*j39[2]);

    //=================================================================//

#endif



    //====================calculate the real angular velocities=======================//
    double w_1, w_2, w_3, w_4, w_5, w_6, w_7, w_8, w_9;
#ifdef WITH_NMPC
    w_1 = mc(0,0); w_2 = mc(1,0); w_3 = mc(2,0); w_4 = mc(3,0); w_5 = mc(4,0);
    w_6 = mc(5,0); w_7 = mc(6,0); w_8 = mc(7,0); w_9 = mc(8,0);
#else
    w_1 = w1; w_2 = w2; w_3 = w3; w_4 = w4; w_5 = w5;
    w_6 = w6; w_7 = w7; w_8 = w8; w_9 = w9;
#endif

   jps[index].calcAngularVelocities(w_1, w_2, w_3, w_4, w_5, w_6, w_7, w_8, w_9);


    cout<<"the real velocity of joint1 :"<<jps[index].dq1<<endl;
    cout<<"the real velocity of joint2 :"<<jps[index].dq2<<endl;
    cout<<"the real velocity of joint3 :"<<jps[index].dq3<<endl;
    cout<<"the real velocity of joint4 :"<<jps[index].dq4<<endl;
    cout<<"the real velocity of joint5 :"<<jps[index].dq5<<endl;
    cout<<"the real velocity of joint6 :"<<jps[index].dq6<<endl;
    cout<<"the real velocity of x :"<<jps[index].dx0<<endl;
    cout<<"the real velocity of y :"<<jps[index].dy0<<endl;
    cout<<"the real velocity of alpha :"<<jps[index].dph<<endl;
    ofstream file0;
    file0.open("/home/weili/catkin_ws/joints.txt",ios::app|ios::out);
    file0<<"the joints of robotic arm are: "<<joint_now[0]<<" , "<<joint_now[1]<<" , "<<joint_now[2]<<" , "
        <<joint_now[3]<<" , "<<joint_now[4]<<" , "<<joint_now[5]<<endl;

    ofstream file;
    file.open("/home/weili/catkin_ws/position_robot.txt",ios::app|ios::out);
    file<<"the pose of robot is: "<<x0<<" , "<<y0<<" , "<<alpha<<endl;
    ofstream file_1;
    file_1.open("/home/weili/catkin_ws/position_end.txt",ios::app|ios::out);
    file_1<<"the pose of robot end is: "<<xe<<" , "<<ye<<" , "<<alphae<<endl;

    ofstream file2;
    file2.open("/home/weili/catkin_ws/distance.txt",ios::app|ios::out);
    file2<<"the distance between the robot and obstacle is:"<<distance_<<" ,index is:"<<index<<endl;

    ofstream file3;
    file3.open("/home/weili/catkin_ws/omega.txt",ios::app|ios::out);
    file3<<"the velocity null-space is: "<<w_1<<" , "<<w_2<<" , "<<w_3<<" , "<<w_4<<" , "<<w_5<<
        " , "<<w_6<<" , "<<w_7<<" , "<<w_8<<" , "<<w_9<<endl;

    ofstream file4;
    file4.open("/home/weili/catkin_ws/velocity.txt",ios::app|ios::out);
    file4<<"the real velocity: "<<jps[index].dq1<<" , "<<jps[index].dq2<<" , "<<jps[index].dq3<<" , "<<jps[index].dq4<<
           " , "<<jps[index].dq5<<" , "<<jps[index].dq6<<" , "<<jps[index].dx0<<" , "<<jps[index].dy0<<" , "<<jps[index].dph<<endl;

    vx=jps[index].dx0*cos(alpha)+jps[index].dy0*sin(alpha);
    vy=-jps[index].dx0*sin(alpha)+jps[index].dy0*cos(alpha);
    w=jps[index].dph;
    cout<<"vx:"<<vx<<", vy:"<<vy<<", vw:"<<w<<endl;

    ofstream file5;
    file5.open("/home/weili/catkin_ws/velocity_base.txt",ios::app|ios::out);
    file5<<"vx:"<<vx<<", vy:"<<vy<<", vw:"<<w<<endl;

    //==========================move the arm===========================//
    robot_state::RobotStatePtr current_state = robot->getCurrentState();
    robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
    cout<<robot_state.joint_state<<endl;
    robot->setNumPlanningAttempts(4);
    robot->setPlannerId("RRTConnectkConfigDefault");
    robot->setStartState(robot_state);
    std::vector<double> joints_temp;
    joints_temp.push_back(joint_now[0]+yita*jps[index].dq1);
    joints_temp.push_back(joint_now[1]+yita*jps[index].dq2);
    joints_temp.push_back(joint_now[2]+yita*jps[index].dq3);
    joints_temp.push_back(joint_now[3]+yita*jps[index].dq4);
    joints_temp.push_back(joint_now[4]+yita*jps[index].dq5);
    joints_temp.push_back(joint_now[5]+yita*jps[index].dq6);
    robot->setJointValueTarget(joints_temp);
    bool success = false;
    success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<"hello world 3"<<endl;
    //robot->execute(plan);
    //cout<<"hello world 4"<<endl;

    std::thread t1(&moveit::planning_interface::MoveGroup::execute, robot, plan);
    //==========================move the base==========================//

    x0_1 = x0 + yita*jps[index].dx0;
    y0_1 = y0 + yita*jps[index].dy0;
    alpha_1 = alpha + yita*jps[index].dph;
    RobotBaseControl rbctrl( n, x0, y0, alpha, x0_1, y0_1, alpha_1, vx, vy, w );

    std::thread t2(&RobotBaseControl::control_base, &rbctrl);
    //==========================stop the base==========================//

    t1.join();
    t2.join();
    joint_now = joints_temp;

    //====================set obstacle to move=========================//
#ifdef FROM_BACK
    //設定cylinder Position
       geometry_msgs::Point cyl_position;
       cyl_position.x = obstacle_pose.position.x;
       cyl_position.y = obstacle_pose.position.y+velocity*yita;
       cyl_position.z = obstacle_pose.position.z;
       //設定cylinder orientation
       geometry_msgs::Quaternion cyl_orientation;
       cyl_orientation.x = obstacle_pose.orientation.x;
       cyl_orientation.y = obstacle_pose.orientation.y;
       cyl_orientation.z = obstacle_pose.orientation.z;
       cyl_orientation.w = obstacle_pose.orientation.w;

       geometry_msgs::Pose cyl_pose;
       cyl_pose.position = cyl_position;
       cyl_pose.orientation = cyl_orientation;

    geometry_msgs::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = velocity*yita;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    modelstate.twist = model_twist;

    modelstate.pose = cyl_pose;

    setmodelstate.request.model_state = modelstate;
#else
    //設定cylinder Position
       geometry_msgs::Point cyl_position;
       cyl_position.x = obstacle_pose.position.x+velocity*yita;
       cyl_position.y = obstacle_pose.position.y;
       cyl_position.z = obstacle_pose.position.z;
       //設定cylinder orientation
       geometry_msgs::Quaternion cyl_orientation;
       cyl_orientation.x = obstacle_pose.orientation.x;
       cyl_orientation.y = obstacle_pose.orientation.y;
       cyl_orientation.z = obstacle_pose.orientation.z;
       cyl_orientation.w = obstacle_pose.orientation.w;

       geometry_msgs::Pose cyl_pose;
       cyl_pose.position = cyl_position;
       cyl_pose.orientation = cyl_orientation;

    geometry_msgs::Twist model_twist;
    model_twist.linear.x = velocity*yita;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    modelstate.twist = model_twist;

    modelstate.pose = cyl_pose;

    setmodelstate.request.model_state = modelstate;
#endif


    if (client.call(setmodelstate))
    {
      ROS_INFO("BRILLIANT!!!");
      ROS_INFO("%f, %f",modelstate.pose.position.x,modelstate.pose.position.y);
    }
    else
    {
      ROS_ERROR("Failed to call service ");
      return 1;
    }
    ros::Duration(1.0).sleep();
    //===============================================//

    xstates.clear();
    jps.clear();
    joint_now.clear();
}


}
