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



int main (int argc, char **argv)
{
  ros::init(argc,argv,"robot_mover_mark_two");
  ros::NodeHandle n;

  ros::Rate loop_rate(20);
  ros::AsyncSpinner spinner(1);
  spinner.start();

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
  robot_state::RobotStatePtr current_state = robot->getCurrentState();
  robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
  cout<<"Print robot state:"<<robot_state.joint_state<<endl;
  robot->setNumPlanningAttempts(4);
  robot->setPlannerId("RRTConnectkConfigDefault");
  robot->setStartState(robot_state);
  std::vector<double> joints;
  joints.push_back(0);
  joints.push_back(-0.93);
  joints.push_back(1.19);
  joints.push_back(-1.82);
  joints.push_back(-1.56);
  joints.push_back(0.163);
  robot->setJointValueTarget(joints);
  bool success = false;
  success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  cout<<"hello world 3"<<endl;
  robot->execute(plan);
  cout<<"hello world 4"<<endl;

  ros::Duration(2.0).sleep();
  current_state = robot->getCurrentState();
  robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
  cout<<"Print robot state:"<<robot_state.joint_state<<endl;
  robot->setNumPlanningAttempts(4);
  robot->setPlannerId("RRTConnectkConfigDefault");
  robot->setStartState(robot_state);
  std::vector<double> joints2;
  joints2.push_back(0.1);
  joints2.push_back(-0.93);
  joints2.push_back(1.29);
  joints2.push_back(-1.82);
  joints2.push_back(-1.56);
  joints2.push_back(0.163);
  robot->setJointValueTarget(joints2);
  success = false;
  success = (robot->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  robot->execute(plan);
  cout<<"hello world 5"<<endl;



  return 0;
}

