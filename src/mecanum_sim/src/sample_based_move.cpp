#include "ros/ros.h"
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


#include <FoliTreeNode.h>
#include <FoliTree.h>
#include <JacobianPoint.h>
#include <time.h>
#define FROM_BACK


using namespace std;
using namespace cv;


//I'm going to implement user-specified twist values later.
const float WHEEL_RAD = 0.18;//0.1016; // meters
const float WHEELBASE = 0.76;//0.466725; // meters
const float TRACK = 0.68;//0.2667; // meters


geometry_msgs::Pose robot_pose;
geometry_msgs::Pose obstacle_pose;
geometry_msgs::Pose end_pose;

double velocity = 0.7;
double yita = 0.1;

double rand01()
{
    return (rand()/((float)RAND_MAX));
}

struct Configure
{
    Configure(double theta1_, double theta2_, double theta3_, double theta4_, double theta5_, double theta6_, double x_, double y_, double alpha_)
        :th1(theta1_),th2(theta2_),th3(theta3_),th4(theta4_),th5(theta5_),th6(theta6_),x0(x_),y0(y_),alph(alpha_){}
    double th1, th2, th3, th4, th5, th6, x0, y0, alph;
};

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
        vw.angular.z = 0;
    }


    base_pub.publish(vw);
}

geometry_msgs::Pose TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose)
{
    Eigen::Quaterniond qo(obj_pose.orientation.w,obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z);
    Eigen::Quaterniond qr(robot_pose.orientation.w,robot_pose.orientation.x,robot_pose.orientation.y,robot_pose.orientation.z);
    Eigen::Quaterniond qo_last = qr.inverse()*qo;
    geometry_msgs::Pose relative;

    double theta = 2*asin(robot_pose.orientation.z);
    double xi = 0.0;
    double yi = 0.0;
    relative.position.x = xi+(obj_pose.position.x-robot_pose.position.x)*cos(theta)+(obj_pose.position.y-robot_pose.position.y)*sin(theta);
    relative.position.y = yi-(obj_pose.position.x-robot_pose.position.x)*sin(theta)+(obj_pose.position.y-robot_pose.position.y)*cos(theta);
    relative.position.z = obj_pose.position.z-0.09;
    relative.orientation.w = qo_last.w();
    relative.orientation.x = qo_last.x();
    relative.orientation.y = qo_last.y();
    relative.orientation.z = qo_last.z();
    return relative;

}

struct Point2D{
    Point2D(){}
    Point2D(double x_, double y_){
        x = x_;
        y = y_;
    }
    double x, y;
};
bool Is_Robot_Colliding_With_Circum(geometry_msgs::Pose rel_desk_pose, geometry_msgs::Pose rel_people_pose)
{
    bool Is_Robot_In_Colliding = false;
    double a, b, r;
    a = 0.80;
    b = 0.88;
    r = 0.35;
    double theta = 2*acos(rel_desk_pose.orientation.w);
    double xB = rel_desk_pose.position.x;
    double yB = rel_desk_pose.position.y;
    double xp = rel_people_pose.position.x;
    double yp = rel_people_pose.position.y;
    Point2D pa_1(a/2, a/2);
    Point2D pa_2(-a/2, a/2);
    Point2D pa_3(-a/2, -a/2);
    Point2D pa_4(a/2, -a/2);
    Point2D pb_1prime, pb_2prime, pb_3prime, pb_4prime;
    Point2D pb_1(b/2, b/2);
    Point2D pb_2(-b/2, b/2);
    Point2D pb_3(-b/2, -b/2);
    Point2D pb_4(b/2, -b/2);
    Point2D pa_1prime, pa_2prime, pa_3prime, pa_4prime;
    pb_1prime.x = (pa_1.x - xB)*cos(theta) + (pa_1.y - yB)*sin(theta);
    pb_1prime.y = (xB - pa_1.x)*sin(theta) - (yB - pa_1.y)*cos(theta);
    pb_2prime.x = (pa_2.x - xB)*cos(theta) + (pa_2.y - yB)*sin(theta);
    pb_2prime.y = (xB - pa_2.x)*sin(theta) - (yB - pa_2.y)*cos(theta);
    pb_3prime.x = (pa_3.x - xB)*cos(theta) + (pa_3.y - yB)*sin(theta);
    pb_3prime.y = (xB - pa_3.x)*sin(theta) - (yB - pa_3.y)*cos(theta);
    pb_4prime.x = (pa_4.x - xB)*cos(theta) + (pa_4.y - yB)*sin(theta);
    pb_4prime.y = (xB - pa_4.x)*sin(theta) - (yB - pa_4.y)*cos(theta);
    pa_1prime.x = pb_1.x*cos(theta) - pb_1.y*sin(theta) + xB;
    pa_1prime.y = pb_1.x*sin(theta) + pb_1.y*cos(theta) + yB;
    pa_2prime.x = pb_2.x*cos(theta) - pb_2.y*sin(theta) + xB;
    pa_2prime.y = pb_2.x*sin(theta) + pb_2.y*cos(theta) + yB;
    pa_3prime.x = pb_3.x*cos(theta) - pb_3.y*sin(theta) + xB;
    pa_3prime.y = pb_3.x*sin(theta) + pb_3.y*cos(theta) + yB;
    pa_4prime.x = pb_4.x*cos(theta) - pb_4.y*sin(theta) + xB;
    pa_4prime.y = pb_4.x*sin(theta) + pb_4.y*cos(theta) + yB;
    if(((abs(xp)<a/2)&&(abs(yp)<(a/2+r)))||((abs(yp)<a/2)&&(abs(xp)<a/2+r))||((abs(xp)-a/2)*(abs(xp)-a/2)+(abs(yp)-a/2)*(abs(yp)-a/2)<r*r))
    {
        Is_Robot_In_Colliding = true;
    }
    else if(((abs(pa_1prime.x)<a/2)&&(abs(pa_1prime.y)<a/2))||((abs(pb_1prime.x)<b/2)&&(abs(pb_1prime.y)<b/2))||((abs(pa_2prime.x)<a/2)&&(abs(pa_2prime.y)<a/2))||((abs(pb_2prime.x)<b/2)&&(abs(pb_2prime.y)<b/2))||((abs(pa_3prime.x)<a/2)&&(abs(pa_3prime.y)<a/2))||((abs(pb_3prime.x)<b/2)&&(abs(pb_3prime.y)<b/2))||((abs(pa_4prime.x)<a/2)&&(abs(pa_4prime.y)<a/2))||((abs(pb_4prime.x)<b/2)&&(abs(pb_4prime.y)<b/2)))
    {
        Is_Robot_In_Colliding = true;
    }
    else
    {
        Is_Robot_In_Colliding = false;
    }
    return Is_Robot_In_Colliding;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sample_based_tcmp");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber sub_joint=n.subscribe("/gazebo/link_states", 10, cb);
    int rl=5;
    while(rl)
    {
      ros::spinOnce();
      loop_rate.sleep();
      rl--;
    }
    double x0, y0, alpha;
    x0 = robot_pose.position.x;
    y0 = robot_pose.position.y;
    alpha = 2*acos(robot_pose.orientation.w);
    cout<<"The robot position :"<<x0<<","<<y0<<","<<alpha<<endl;

    //============================init KDL, robot and scene=============================//
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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    //------------init robot------------//

    moveit::planning_interface::MoveGroup *robot = new moveit::planning_interface::MoveGroup("manipulator");
    moveit::planning_interface::MoveGroup::Plan plan;
    moveit_msgs::RobotState robot_state;
    cout<<"get robotic arm move group"<<endl;
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
    cout<<"initial near point in the arm."<<endl;


    //==============================Planning Loop===================================//

    double th_max = 2*M_PI;
    double th_min = -2*M_PI;
    double x_max = 8;
    double x_min = -8;
    double x_o, y_o, z_o;
    int Num_try = 50;
    double Time = 1.7;
    double step = 0.12;
    double time_step = 0.1;
    time_step = yita;
    int Iter = int(Time/time_step);
    FoliTree pTree;
    FoliTreeNode* pTreeAncestry = new FoliTreeNode();
    vector<double> joints = robot->getCurrentJointValues();
    cout<<"the robotic joints:"<<joints[0]<<","<<joints[1]<<","<<joints[2]<<","<<joints[3]<<","<<joints[4]<<","<<joints[5]<<endl;
    pTreeAncestry->th1 = joints[0];
    pTreeAncestry->th2 = joints[1];
    pTreeAncestry->th3 = joints[2];
    pTreeAncestry->th4 = joints[3];
    pTreeAncestry->th5 = joints[4];
    pTreeAncestry->th6 = joints[5];
    pTreeAncestry->x0 = x0;
    pTreeAncestry->y0 = y0;
    pTreeAncestry->alph = alpha;
    pTreeAncestry->level = 0;
    pTree.ancestry = pTreeAncestry;
    pTree.subouter.push_back(pTreeAncestry);
    cout<<"==================Begin Loop====================="<<endl;
    for(int i = 1; i <= Iter; i ++)
    {
        for(int num_try = 0; num_try < Num_try; num_try ++)
        {
            double th1_rand = rand01()*(th_max - th_min) + th_min;
            cout<<"rand01:"<<rand01()<<"th1_rand:"<<th1_rand<<endl;
            double th2_rand = rand01()*(th_max - th_min) + th_min;
            double th3_rand = rand01()*(th_max - th_min) + th_min;
            double th4_rand = rand01()*(th_max - th_min) + th_min;
            double th5_rand = rand01()*(th_max - th_min) + th_min;
            double th6_rand = rand01()*(th_max - th_min) + th_min;
            double x_rand = rand01()*(x_max - x_min) + x_min;
            cout<<"rand01:"<<rand01()<<"x_rand:"<<x_rand<<endl;
            double y_rand = rand01()*(x_max - x_min) + x_min;
            double alph_rand = rand01()*(th_max - th_min) + th_min;
            double dist_to_rand = 10000;
            int index_of_nearest = 0;

            for(int j = 0; j < pTree.subouter.size(); j ++)
            {
                vector<double> joint_now;
                joint_now.push_back(pTree.subouter[j]->th1);    joint_now.push_back(pTree.subouter[j]->th2);    joint_now.push_back(pTree.subouter[j]->th3);
                joint_now.push_back(pTree.subouter[j]->th4);    joint_now.push_back(pTree.subouter[j]->th5);    joint_now.push_back(pTree.subouter[j]->th6);
                x0 = pTree.subouter[j]->x0;    y0 = pTree.subouter[j]->y0;    alpha = pTree.subouter[j]->alph;
                double dist_to_rand_ = sqrt((th1_rand-joint_now[0])*(th1_rand-joint_now[0])+(th2_rand-joint_now[1])*(th2_rand-joint_now[1])+(th3_rand-joint_now[2])*(th3_rand-joint_now[2])
                        +(th4_rand-joint_now[3])*(th4_rand-joint_now[3])+(th5_rand-joint_now[4])*(th5_rand-joint_now[4])+(th6_rand-joint_now[5])*(th6_rand-joint_now[5])+(x_rand-x0)*(x_rand-x0)
                        +(y_rand-y0)*(y_rand-y0)+(alph_rand-alpha)*(alph_rand-alpha));
                if(dist_to_rand_<dist_to_rand)
                {
                    index_of_nearest = j;
                    dist_to_rand = dist_to_rand_;
                }
            }
            cout<<"Find the nearest node to extend tree......"<<endl;
            vector<double> joint_nearest;
            joint_nearest.push_back(pTree.subouter[index_of_nearest]->th1);    joint_nearest.push_back(pTree.subouter[index_of_nearest]->th2);    joint_nearest.push_back(pTree.subouter[index_of_nearest]->th3);
            joint_nearest.push_back(pTree.subouter[index_of_nearest]->th4);    joint_nearest.push_back(pTree.subouter[index_of_nearest]->th5);    joint_nearest.push_back(pTree.subouter[index_of_nearest]->th6);
            x0 = pTree.subouter[index_of_nearest]->x0;    y0 = pTree.subouter[index_of_nearest]->y0;    alpha = pTree.subouter[index_of_nearest]->alph;

            double delta_wjnt1_new = (th1_rand - joint_nearest[0])*step/dist_to_rand;
            double delta_wjnt2_new = (th2_rand - joint_nearest[1])*step/dist_to_rand;
            double delta_wjnt3_new = (th3_rand - joint_nearest[2])*step/dist_to_rand;
            double delta_wjnt4_new = (th4_rand - joint_nearest[3])*step/dist_to_rand;
            double delta_wjnt5_new = (th5_rand - joint_nearest[4])*step/dist_to_rand;
            double delta_wjnt6_new = (th6_rand - joint_nearest[5])*step/dist_to_rand;
            double delta_wx_new = (x_rand - x0)*step/dist_to_rand;
            double delta_wy_new = (y_rand - y0)*step/dist_to_rand;
            double delta_walph_new = (alph_rand - alpha)*step/dist_to_rand;

            JacobianPoint jp17(np17.index_of_arm, np17.x, np17.y, np17.z, x0, y0, alpha, joint_nearest[0], joint_nearest[1], joint_nearest[2], joint_nearest[3], joint_nearest[4], joint_nearest[5]);
            jp17.calcJacobianWhole();
            jp17.calcJacobianNear();
            jp17.calcJacobianStar();
            jp17.calcAngularVelocities(delta_wjnt1_new, delta_wjnt2_new, delta_wjnt3_new, delta_wjnt4_new, delta_wjnt5_new, delta_wjnt6_new, delta_wx_new, delta_wy_new, delta_walph_new);
            cout<<"have calculated the real velocity of robot......"<<endl;

            double dist_to_nearest = sqrt((jp17.dq1)*(jp17.dq1)+(jp17.dq2)*(jp17.dq2)+(jp17.dq3)*(jp17.dq3)+(jp17.dq4)*(jp17.dq4)
                                          +(jp17.dq5)*(jp17.dq5)+(jp17.dq6)*(jp17.dq6)+(jp17.dx0)*(jp17.dx0)+(jp17.dy0)*(jp17.dy0)+(jp17.dph)*(jp17.dph));
            double qjnt1_new = joint_nearest[0] + jp17.dq1;
            double qjnt2_new = joint_nearest[1] + jp17.dq2;
            double qjnt3_new = joint_nearest[2] + jp17.dq3;
            double qjnt4_new = joint_nearest[3] + jp17.dq4;
            double qjnt5_new = joint_nearest[4] + jp17.dq5;
            double qjnt6_new = joint_nearest[5] + jp17.dq6;
            double qx_new = x0 + jp17.dx0;
            double qy_new = y0 + jp17.dy0;
            double qalph_new = alpha + jp17.dph;
            geometry_msgs::Pose abs_robot_pose;
            abs_robot_pose.position.x = qx_new;
            abs_robot_pose.position.y = qy_new;
            abs_robot_pose.position.z = 0;
            abs_robot_pose.orientation.w = cos(qalph_new/2.0);
            abs_robot_pose.orientation.x = 0.0;
            abs_robot_pose.orientation.y = 0.0;
            abs_robot_pose.orientation.z = sin(qalph_new/2.0);

            //------------------------desk---------------------------//
            moveit_msgs::CollisionObject box;
            box.header.frame_id = "base_footprint";
            box.id = "box";
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.913;
            primitive.dimensions[1] = 0.913;
            primitive.dimensions[2] = 0.755;

            geometry_msgs::Pose abs_desk_pose;
            abs_desk_pose.position.x = 0.014;
            abs_desk_pose.position.y = 1.27919;
            abs_desk_pose.position.z = 0;
            abs_desk_pose.orientation.w = 1.0;
            abs_desk_pose.orientation.x = 0.0;
            abs_desk_pose.orientation.y = 0.0;
            abs_desk_pose.orientation.z = 0.0;

            geometry_msgs::Pose rel_desk_pose;
            rel_desk_pose = TransformFromObjToRobot(abs_desk_pose,abs_robot_pose);
            cout<<"The absolute pose of robot is :"<<abs_robot_pose<<endl;
            cout<<"The relative pose of desk to the robot is :"<<rel_desk_pose<<endl;
            box.primitives.push_back(primitive);
            box.primitive_poses.push_back(rel_desk_pose);
            box.operation = box.ADD;
            //--------------------------------------------------------//

            //------------------------people--------------------------//
            moveit_msgs::CollisionObject cyl1;
            cyl1.header.frame_id = "base_footprint";
            cyl1.id = "cylinder1";
            shape_msgs::SolidPrimitive primitive1;
            primitive1.type = primitive1.CYLINDER;
            primitive1.dimensions.resize(3);
            primitive1.dimensions[0] = 1.8;
            primitive1.dimensions[1] = 0.35;

            geometry_msgs::Pose abs_people_pose = obstacle_pose;

            #ifdef FROM_BACK
            abs_people_pose.position.x = obstacle_pose.position.x;
            abs_people_pose.position.y = obstacle_pose.position.y + velocity*time_step*i;
            abs_people_pose.position.z = obstacle_pose.position.z;
            #else
            abs_people_pose.position.x = obstacle_pose.position.x + velocity*time_step*i;
            abs_people_pose.position.y = obstacle_pose.position.y;
            abs_people_pose.position.z = obstacle_pose.position.z;
            #endif

            geometry_msgs::Pose rel_people_pose;
            rel_people_pose = TransformFromObjToRobot(abs_people_pose,abs_robot_pose);
            cout<<"The relative pose of people to the robot is :"<<rel_people_pose<<endl;
            cyl1.primitives.push_back(primitive1);
            cyl1.primitive_poses.push_back(rel_people_pose);
            cyl1.operation = cyl1.ADD;
            //--------------------------------------------------------//

            std::vector<moveit_msgs::CollisionObject> collision_objects;
            collision_objects.push_back(box);
            collision_objects.push_back(cyl1);
            moveit_msgs::PlanningScene planning_scene;
            planning_scene.world.collision_objects.push_back(box);
            planning_scene.world.collision_objects.push_back(cyl1);
            planning_scene.is_diff = true;
            planning_scene_interface.addCollisionObjects(collision_objects);
            planning_scene_diff_publisher.publish(planning_scene);

            string word[6]={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
            std::vector<std::string> joint_names_0(word,word+6);

            planning_scene::PlanningScenePtr current_scene = scene->getPlanningScene();
            robot_state::RobotState current_robot_state = current_scene->getCurrentState();
            robot->setStartState(current_robot_state);
            cout<<"set the colliding objects in the world......"<<endl;
            //const robot_state::JointModelGroup *joint_model_group = current_robot_state.getJointModelGroup(robot->getName());
            vector<double> back_joints;
            vector<double> forward_joints;

            back_joints.push_back(joint_nearest[0]);
            back_joints.push_back(joint_nearest[1]);
            back_joints.push_back(joint_nearest[2]);
            back_joints.push_back(joint_nearest[3]);
            back_joints.push_back(joint_nearest[4]);
            back_joints.push_back(joint_nearest[5]);

            forward_joints.push_back(qjnt1_new);
            forward_joints.push_back(qjnt2_new);
            forward_joints.push_back(qjnt3_new);
            forward_joints.push_back(qjnt4_new);
            forward_joints.push_back(qjnt5_new);
            forward_joints.push_back(qjnt6_new);

            current_robot_state.copyJointGroupPositions( robot->getName(), back_joints);
            current_robot_state.setVariablePositions( joint_names_0, forward_joints);
            //bool Is_Colliding = current_scene->isStateColliding( current_robot_state );
            bool Is_Colliding = Is_Robot_Colliding_With_Circum(rel_desk_pose, rel_people_pose);
            if(Is_Colliding)
            {
                cout<<"the new node is in colliding!"<<endl;
                continue;
            }
            else
            {
                cout<<"the new node is not in colliding!"<<endl;
                //boost::shared_ptr<FoliTreeNode> pTreeNewNode(new FoliTreeNode);
                FoliTreeNode* pTreeNewNode = new FoliTreeNode();
                pTreeNewNode->th1 = qjnt1_new;
                pTreeNewNode->th2 = qjnt2_new;
                pTreeNewNode->th3 = qjnt3_new;
                pTreeNewNode->th4 = qjnt4_new;
                pTreeNewNode->th5 = qjnt5_new;
                pTreeNewNode->th6 = qjnt6_new;
                pTreeNewNode->x0 = qx_new;
                pTreeNewNode->y0 = qy_new;
                pTreeNewNode->alph = qalph_new;
                pTreeNewNode->dw1 = delta_wjnt1_new;
                pTreeNewNode->dw2 = delta_wjnt2_new;
                pTreeNewNode->dw3 = delta_wjnt3_new;
                pTreeNewNode->dw4 = delta_wjnt4_new;
                pTreeNewNode->dw5 = delta_wjnt5_new;
                pTreeNewNode->dw6 = delta_wjnt6_new;
                pTreeNewNode->dw7 = delta_wx_new;
                pTreeNewNode->dw8 = delta_wy_new;
                pTreeNewNode->dw9 = delta_walph_new;
                pTreeNewNode->Father = pTree.subouter[index_of_nearest];
                pTreeNewNode->dist_to_init = pTree.subouter[index_of_nearest]->dist_to_init + dist_to_nearest;

                //================================calculate the distance between the new node and people========================//

#ifdef FROM_BACK
    x_o = abs_people_pose.position.x;//-1.0;
    y_o = abs_people_pose.position.y+0.2;//0.5;
    z_o = 0.5;//1.5;
#else
    x_o = abs_people_pose.position.x+0.2;//-1.0;
    y_o = abs_people_pose.position.y;//0.5;
    z_o = 1.0;//0.5;//1.5;
#endif

    cout<<"the obstacle x :"<<x_o<<", y_o :"<<y_o<<", z_o :"<<z_o<<endl;

    //==============================================================//

    x0 = qx_new;
    y0 = qy_new;
    alpha = qalph_new;

    cout<<"the pose of robot is: x0:"<<x0<<" ,y0:"<<y0<<", alpha:"<<alpha<<endl;
    vector<JacobianPoint> jps;
    JacobianPoint jp1(np1.index_of_arm, np1.x, np1.y, np1.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp1);
    JacobianPoint jp2(np2.index_of_arm, np2.x, np2.y, np2.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp2);
    JacobianPoint jp3(np3.index_of_arm, np3.x, np3.y, np3.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp3);
    JacobianPoint jp4(np4.index_of_arm, np4.x, np4.y, np4.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp4);
    JacobianPoint jp5(np5.index_of_arm, np5.x, np5.y, np5.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp5);
    JacobianPoint jp6(np6.index_of_arm, np6.x, np6.y, np6.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp6);
    JacobianPoint jp7(np7.index_of_arm, np7.x, np7.y, np7.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp7);
    JacobianPoint jp8(np8.index_of_arm, np8.x, np8.y, np8.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp8);
    JacobianPoint jp9(np9.index_of_arm, np9.x, np9.y, np9.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp9);
    JacobianPoint jp10(np10.index_of_arm, np10.x, np10.y, np10.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp10);
    JacobianPoint jp11(np11.index_of_arm, np11.x, np11.y, np11.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp11);
    JacobianPoint jp12(np12.index_of_arm, np12.x, np12.y, np12.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp12);
    JacobianPoint jp13(np13.index_of_arm, np13.x, np13.y, np13.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp13);
    JacobianPoint jp14(np14.index_of_arm, np14.x, np14.y, np14.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp14);
    JacobianPoint jp15(np15.index_of_arm, np15.x, np15.y, np15.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp15);
    JacobianPoint jp16(np16.index_of_arm, np16.x, np16.y, np16.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp16);
    JacobianPoint jp17_(np17.index_of_arm, np17.x, np17.y, np17.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp17_);
    JacobianPoint jp18(np18.index_of_arm, np18.x, np18.y, np18.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp18);
    JacobianPoint jp19(np19.index_of_arm, np19.x, np19.y, np19.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp19);
    JacobianPoint jp20(np20.index_of_arm, np20.x, np20.y, np20.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp20);
    JacobianPoint jp21(np21.index_of_arm, np21.x, np21.y, np21.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp21);
    JacobianPoint jp22(np22.index_of_arm, np22.x, np22.y, np22.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp22);
    JacobianPoint jp23(np23.index_of_arm, np23.x, np23.y, np23.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp23);
    JacobianPoint jp24(np24.index_of_arm, np24.x, np24.y, np24.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp24);
    JacobianPoint jp25(np25.index_of_arm, np25.x, np25.y, np25.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp25);
    JacobianPoint jp26(np26.index_of_arm, np26.x, np26.y, np26.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp26);
    JacobianPoint jp27(np27.index_of_arm, np27.x, np27.y, np27.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp27);
    JacobianPoint jp28(np28.index_of_arm, np28.x, np28.y, np28.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp28);
    JacobianPoint jp29(np29.index_of_arm, np29.x, np29.y, np29.z, x0, y0, alpha, forward_joints[0], forward_joints[1], forward_joints[2], forward_joints[3], forward_joints[4], forward_joints[5]);
    jps.push_back(jp29);
    vector<JacobianPoint>::iterator iter;
    double dist_to_obstacle = 10000;
    int index_to_obstacle = 0;

    int i = 0;
    for(iter = jps.begin(); iter!= jps.end(); iter++)
    {
        double d = iter->calcDistanceToObstacle(x_o, y_o, z_o);
        if(d<dist_to_obstacle)
        {
            dist_to_obstacle = d;
            index_to_obstacle = i;
        }
        i++;
    }
    pTreeNewNode->dist_to_obstacle = dist_to_obstacle;
    pTreeNewNode->clearest_point_index = i;

                //==============================================================================================================//

                pTree.subouter[index_of_nearest]->Sons.push_back(pTreeNewNode);
                pTree.outermost.push_back(pTreeNewNode);
                jps.clear();
            }
            //从世界中移除物体
            ROS_INFO("Remove the object from the world");
            std::vector<std::string> object_ids;
            object_ids.push_back(box.id);
            object_ids.push_back(cyl1.id);
            planning_scene_interface.removeCollisionObjects(object_ids);
            cout<<"==========================the number nodes added in the "<<i-1<<" th layer of the FoliTree is: "<<pTree.subouter.size()<<"============================"<<endl;

            cout<<"==========================the number nodes added in the "<<i<<" th layer of the FoliTree is: "<<pTree.outermost.size()<<"============================"<<endl;

        }


        pTree.subouter.clear();
//        pTree.subouter = pTree.outermost;
        pTree.subouter.swap(pTree.outermost);
        //pTree.subouter.assign(pTree.outermost.begin(), pTree.outermost.end());
        pTree.outermost.clear();
        cout<<"switch outermost and subouter, and now the size of subouter is :"<<pTree.subouter.size()<<", and the size of outermost is :"<<pTree.outermost.size()<<endl;

    }

    //=================================Begin execute path====================================//
    double min_dist_path = 100000;
    int min_dist_index = 0;
    double omega_length = 0.5;
    double omega_distance = 0.5;
    for(int i = 0; i < pTree.subouter.size(); i ++)
    {
        double d = omega_distance/pTree.subouter[i]->dist_to_obstacle;//omega_length*pTree.subouter[i]->dist_to_init + omega_distance/pTree.subouter[i]->dist_to_obstacle;
        if(d < min_dist_path)
        {
            min_dist_path = d;
            min_dist_index = i;
        }
    }
    pTree.nearestgrandson = pTree.subouter[min_dist_index];
    pTree.getShortestPath();


  for(int i = 1; i < pTree.shortestPath.size(); i ++)
  {
      sub_joint=n.subscribe("/gazebo/link_states", 10, cb);
      int rl=5;
      while(rl)
      {
         ros::spinOnce();
         loop_rate.sleep();
         rl--;
      }


      //=========================record the data================================//
      ofstream file0;
      file0.open("/home/weili/catkin_ws/joints.txt",ios::app|ios::out);
      file0<<"the joints of robotic arm are: "<<pTree.shortestPath[i-1]->th1<<" , "<<pTree.shortestPath[i-1]->th2<<" , "<<pTree.shortestPath[i-1]->th3<<" , "
          <<pTree.shortestPath[i-1]->th4<<" , "<<pTree.shortestPath[i-1]->th5<<" , "<<pTree.shortestPath[i-1]->th6<<endl;

      ofstream file;
      file.open("/home/weili/catkin_ws/position_robot.txt",ios::app|ios::out);
      double pha = 2*acos(robot_pose.orientation.w);
      file<<"the pose of robot is: "<<robot_pose.position.x<<" , "<<robot_pose.position.y<<" , "<<pha<<endl;
      ofstream file_1;
      file_1.open("/home/weili/catkin_ws/position_end.txt",ios::app|ios::out);
      double phae = 2*cos(end_pose.orientation.w);
      file_1<<"the pose of robot end is: "<<end_pose.position.x<<" , "<<end_pose.position.y<<" , "<<phae<<endl;

      ofstream file2;
      file2.open("/home/weili/catkin_ws/distance.txt",ios::app|ios::out);
      file2<<"the distance between the robot and obstacle is:"<<pTree.shortestPath[i]->dist_to_obstacle<<" ,index is:"<<pTree.shortestPath[i]->clearest_point_index<<endl;

      double w_1 = pTree.shortestPath[i]->dw1/time_step;
      double w_2 = pTree.shortestPath[i]->dw2/time_step;
      double w_3 = pTree.shortestPath[i]->dw3/time_step;
      double w_4 = pTree.shortestPath[i]->dw4/time_step;
      double w_5 = pTree.shortestPath[i]->dw5/time_step;
      double w_6 = pTree.shortestPath[i]->dw6/time_step;
      double w_7 = pTree.shortestPath[i]->dw7/time_step;
      double w_8 = pTree.shortestPath[i]->dw8/time_step;
      double w_9 = pTree.shortestPath[i]->dw9/time_step;
      ofstream file3;
      file3.open("/home/weili/catkin_ws/omega.txt",ios::app|ios::out);
      file3<<"the velocity null-space is: "<<w_1<<" , "<<w_2<<" , "<<w_3<<" , "<<w_4<<" , "<<w_5<<
          " , "<<w_6<<" , "<<w_7<<" , "<<w_8<<" , "<<w_9<<endl;
      cout<<"the velocity null-space is: "<<w_1<<" , "<<w_2<<" , "<<w_3<<" , "<<w_4<<" , "<<w_5<<
            " , "<<w_6<<" , "<<w_7<<" , "<<w_8<<" , "<<w_9<<endl;

      double v_1 = (pTree.shortestPath[i]->th1 - pTree.shortestPath[i-1]->th1)/time_step;
      double v_2 = (pTree.shortestPath[i]->th2 - pTree.shortestPath[i-1]->th2)/time_step;
      double v_3 = (pTree.shortestPath[i]->th3 - pTree.shortestPath[i-1]->th3)/time_step;
      double v_4 = (pTree.shortestPath[i]->th4 - pTree.shortestPath[i-1]->th4)/time_step;
      double v_5 = (pTree.shortestPath[i]->th5 - pTree.shortestPath[i-1]->th5)/time_step;
      double v_6 = (pTree.shortestPath[i]->th6 - pTree.shortestPath[i-1]->th6)/time_step;
      double v_7 = (pTree.shortestPath[i]->x0 - pTree.shortestPath[i-1]->x0)/time_step;
      double v_8 = (pTree.shortestPath[i]->y0 - pTree.shortestPath[i-1]->y0)/time_step;
      double v_9 = (pTree.shortestPath[i]->alph - pTree.shortestPath[i-1]->alph)/time_step;
      ofstream file4;
      file4.open("/home/weili/catkin_ws/velocity.txt",ios::app|ios::out);
      file4<<"the real velocity: "<<v_1<<" , "<<v_2<<" , "<<v_3<<" , "<<v_4<<
             " , "<<v_5<<" , "<<v_6<<" , "<<v_7<<" , "<<v_8<<" , "<<v_9<<endl;
      cout<<"the real velocity: "<<v_1<<" , "<<v_2<<" , "<<v_3<<" , "<<v_4<<
            " , "<<v_5<<" , "<<v_6<<" , "<<v_7<<" , "<<v_8<<" , "<<v_9<<endl;


      alpha = pTree.shortestPath[i-1]->alph;
      double vx = v_7*cos(alpha) + v_8*sin(alpha);
      double vy = -v_7*sin(alpha) + v_8*cos(alpha);
      double w = v_9;
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
    joints_temp.push_back(pTree.shortestPath[i]->th1);
    joints_temp.push_back(pTree.shortestPath[i]->th2);
    joints_temp.push_back(pTree.shortestPath[i]->th3);
    joints_temp.push_back(pTree.shortestPath[i]->th4);
    joints_temp.push_back(pTree.shortestPath[i]->th5);
    joints_temp.push_back(pTree.shortestPath[i]->th6);
    robot->setJointValueTarget(joints_temp);
    bool success = false;
    success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<"//---------------move the arm.--------------//"<<endl;
    //robot->execute(plan);
    //cout<<"hello world 4"<<endl;

    std::thread t1(&moveit::planning_interface::MoveGroup::execute, robot, plan);
    //==========================move the base==========================//

    RobotBaseControl rbctrl( n, pTree.shortestPath[i-1]->x0, pTree.shortestPath[i-1]->y0, pTree.shortestPath[i-1]->alph, pTree.shortestPath[i]->x0, pTree.shortestPath[i]->y0, pTree.shortestPath[i]->alph, vx, vy, w );
    cout<<"//---------------move the base.----------------//"<<endl;
    std::thread t2(&RobotBaseControl::control_base, &rbctrl);
    //==========================stop the base==========================//

    t1.join();
    t2.join();
    //====================set obstacle to move=========================//

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

//  cout<<"the obstacle x :"<<x_o<<", y_o :"<<y_o<<", z_o :"<<z_o<<endl;
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
//       cout<<"the pose of people now :"<<obstacle_pose.position.x<<", "<<obstacle_pose.position.y<<endl;
//       cout<<"the pose of people next :"<<cyl_position.x<<", "<<cyl_position.y<<endl;
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
      cout<<"//-----------------move the people.------------------//"<<endl;
      //ROS_INFO("%f, %f",modelstate.pose.position.x,modelstate.pose.position.y);
    }
    else
    {
      ROS_ERROR("Failed to call service ");
      return 1;
    }
    ros::Duration(1.0).sleep();
    //===============================================//


  }


}
