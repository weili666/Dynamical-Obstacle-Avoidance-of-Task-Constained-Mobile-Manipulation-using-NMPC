#include "ros/ros.h"
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
using namespace std;
//I'm going to implement user-specified twist values later.
const float WHEEL_RAD = 0.18;//0.1016; // meters
const float WHEELBASE = 0.76;//0.466725; // meters
const float TRACK = 0.68;//0.2667; // meters
geometry_msgs::Pose robot_pose;



void cb(const gazebo_msgs::LinkStatesConstPtr & link_msgs)
{
    robot_pose = link_msgs->pose[1];
}
void cb_las_1(const sensor_msgs::LaserScanPtr & laser1_msgs)
{
    int ranges = laser1_msgs->ranges.size();
    cout<<"ranges :"<<ranges<<endl;
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
    ~RobotBaseControl(){}
    void cb2(const gazebo_msgs::LinkStatesConstPtr & link_msgs);
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
        vw.linear.x = vx_base;
    }
    else
    {
        vw.linear.x = 0;
    }
    if(abs(delta_y_rel_temp)<abs(delta_y_rel))
    {
        vw.linear.y = vy_base;
    }
    else
    {
        vw.linear.y = 0;
    }
    if(abs(alpha1-alpha)<abs(alpha_1-alpha))
    {
        vw.angular.z = w_base*180/M_PI;
    }
    else
    {
        vw.angular.z = 0;
    }

    cout<<"*******the origin x is :"<<x0<<", y is :"<<y0<<", alpha is :"<<alpha<<endl;
    cout<<"the target x is :"<<x0_1<<", y is :"<<y0_1<<", alpha is :"<<alpha_1<<endl;
    cout<<"the position now x is :"<<x1<<", y is :"<<y1<<", alpha is :"<<alpha1<<endl;
    cout<<"vx is :"<<vw.linear.x<<", vy is :"<<vw.linear.y<<", angular velocity is :"<<vw.angular.z<<endl;
    base_pub.publish(vw);
//    file.close();
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"robot_mover_mark_two");
  ros::NodeHandle n;
  ros::Publisher fl_pub = n.advertise<std_msgs::Float32>("front_left",1,true);
  ros::Publisher fr_pub = n.advertise<std_msgs::Float32>("front_right",1,true);
  ros::Publisher bl_pub = n.advertise<std_msgs::Float32>("back_left",1,true);
  ros::Publisher br_pub = n.advertise<std_msgs::Float32>("back_right",1,true);
  ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
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

  double vx,vy,w;
  vx=0.0;vy=0.0;w=1.57/2;

  geometry_msgs::Twist vw;
  vw.linear.x = vx;
  vw.linear.y = vy;
  vw.linear.z = 0;
  vw.angular.x = 0;
  vw.angular.y = 0;
  vw.angular.z = w;

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
  cout<<robot_state.joint_state<<endl;
  robot->setNumPlanningAttempts(4);
  robot->setPlannerId("RRTConnectkConfigDefault");
  robot->setStartState(robot_state);
  std::vector<double> joints;
  joints.push_back(0);
  joints.push_back(-1.14);
  joints.push_back(1.4);
  joints.push_back(-1.82);
  joints.push_back(-1.56);
  joints.push_back(0.163);
  robot->setJointValueTarget(joints);
  bool success = false;
  success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  cout<<"hello world 3"<<endl;
  robot->execute(plan);
  cout<<"hello world 4"<<endl;

//  base_pub.publish(vw);
//  sleep(1);
  //ros::Duration(90.0).sleep();
  //==========================move the base==========================//

  double yita = 2;
  double x0_1 = x0 + yita*vx;
  double y0_1 = y0 + yita*vy;
  double alpha_1 = alpha + yita*w;
  RobotBaseControl rbctrl( n, x0, y0, alpha, x0_1, y0_1, alpha_1, vx, vy, w );

  vw.linear.x = 0;
  vw.linear.y = 0;
  vw.linear.z = 0;
  vw.angular.x = 0;
  vw.angular.y = 0;
  vw.angular.z = 0;

  base_pub.publish(vw);

//  current_state = robot->getCurrentState();
//  cout<<current_state<<endl;
//  cout<<"hello world 2"<<endl;
//  robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
//  cout<<robot_state.joint_state<<endl;
//  robot->setNumPlanningAttempts(4);
//  robot->setPlannerId("RRTConnectkConfigDefault");
//  robot->setStartState(robot_state);
//  std::vector<double> joints2;
//  joints2.push_back(0.1);
//  joints2.push_back(-0.93);
//  joints2.push_back(1.29);
//  joints2.push_back(-1.82);
//  joints2.push_back(-1.56);
//  joints2.push_back(0.163);
//  robot->setJointValueTarget(joints2);
//  success = false;
//  success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  robot->execute(plan);
//  cout<<"hello world 5"<<endl;



  return 0;  
}

