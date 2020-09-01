#include "ros/ros.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/LaserScan.h>
#include <mecanum_sim/DRLStep.h>
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
#include <boost/shared_ptr.hpp>
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
#include <iostream>

//I'm going to implement user-specified twist values later.
const float WHEEL_RAD = 0.18;//0.1016; // meters
const float WHEELBASE = 0.76;//0.466725; // meters
const float TRACK = 0.68;//0.2667; // meters

using namespace std;
//typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
//typedef std::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;


class StepServer
{
public:
    StepServer(const ros::NodeHandle& n, planning_scene_monitor::PlanningSceneMonitorPtr scene, moveit::planning_interface::MoveGroup *robot, boost::shared_ptr<KDL::ChainFkSolverPos>  jnt_to_pose_solver)
    {
        nh = n;
        scene_ = scene;
        robot_arm = robot;
        jnt_to_pose_solver_ = jnt_to_pose_solver;

        obj_pose.position.x=0;
        obj_pose.position.y=1.15;
        obj_pose.position.z=0.85;
        obj_pose.orientation.w = 1.0;
        obj_pose.orientation.x = 0.0;
        obj_pose.orientation.y = 0.0;
        obj_pose.orientation.z = 0.0;
        ros::ServiceServer service = nh.advertiseService("step_drl", &StepServer::step, this);
        pa_vec.push_back(pa_after);
        cout<<"Now deep reinforcement learning steps:"<<num_steps<<endl;
        ROS_INFO("Ready to step forward deep reinforcement learning.");
        ros::spin();
    }
    ~StepServer(){}
    static int num_steps;
    void cb_pr(const gazebo_msgs::LinkStatesConstPtr & link_msgs);
    void cb_af(const gazebo_msgs::LinkStatesConstPtr & link_msgs);
    bool step(mecanum_sim::DRLStep::Request &req,mecanum_sim::DRLStep::Response &res);
    bool create_motion_plan(const std::vector<double>& joints);
    geometry_msgs::Pose TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose);
    void cb_las_1(const sensor_msgs::LaserScanPtr & laser1_msgs);
    void cb_las_2(const sensor_msgs::LaserScanPtr & laser2_msgs);

private:
    ros::NodeHandle nh;
    PositionAll pa_prior;
    PositionAll pa_theory;
    PositionAll pa_after;
    vector<PositionAll> pa_vec;
    vector<double> pa_grasp;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_;
    moveit::planning_interface::MoveGroup *robot_arm;
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    geometry_msgs::Pose obj_pose;
    sensor_msgs::LaserScan lsr1;
    sensor_msgs::LaserScan lsr2;
    int ranges;
    double omega1;
    double omega2;
    double delta_grasp_angle;
    double G;
    double d_1;
    double d_2;
    double d_3;
    double d_4;
    double d_5;
    double nx1,ny1;
    double nx2,ny2;
    double nx3,ny3;
    double nx4,ny4;
    double nx5,ny5;

};

int StepServer::num_steps = 0;

bool StepServer::create_motion_plan(const std::vector<double> &joints)
{
    moveit::planning_interface::MoveGroup::Plan plan;
    moveit_msgs::RobotState robot_state;
    //cout<<"hello world 1"<<endl;
    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();

    //cout<<"hello world 2"<<endl;
    robot_arm->setNumPlanningAttempts(4);
    robot_arm->setPlannerId("RRTConnectkConfigDefault");
    robot_arm->setStartState(current_robot_state);
    robot_arm->setJointValueTarget(joints);
    //bool success = false;
    //success = (robot_arm->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    robot_arm->plan(plan);
    //cout<<"success:"<<success<<endl;
    robot_arm->execute(plan);
    cout<<"hello world 3"<<endl;
    return true;
}

void StepServer::cb_pr(const gazebo_msgs::LinkStatesConstPtr & link_msgs)
{
    //cout<<"The prious pose of robot:"<<endl;
    //cout<<link_msgs->pose[10];
    pa_prior.desk_pose=link_msgs->pose[8];
    pa_prior.people1_pose=link_msgs->pose[9];
    pa_prior.people2_pose=link_msgs->pose[10];
    pa_prior.people3_pose=link_msgs->pose[11];
    pa_prior.people4_pose=link_msgs->pose[12];
    pa_prior.people5_pose=link_msgs->pose[13];
    pa_prior.robot_pose=link_msgs->pose[1];
    pa_prior.arm_pose=link_msgs->pose[2];
}

void StepServer::cb_af(const gazebo_msgs::LinkStatesConstPtr & link_msgs)
{
    //cout<<"The after pose of robot:"<<endl;
    //cout<<link_msgs->pose[10];
    pa_after.desk_pose=link_msgs->pose[8];
    pa_after.people1_pose=link_msgs->pose[9];
    pa_after.people2_pose=link_msgs->pose[10];
    pa_after.people3_pose=link_msgs->pose[11];
    pa_after.people4_pose=link_msgs->pose[12];
    pa_after.people5_pose=link_msgs->pose[13];
    pa_after.robot_pose=link_msgs->pose[1];
    pa_after.arm_pose=link_msgs->pose[2];
}

void StepServer::cb_las_1(const sensor_msgs::LaserScanPtr & laser1_msgs)
{
    //cout<<"get the laser data 1"<<endl;
    ranges = laser1_msgs->ranges.size();
    lsr1.header.stamp = laser1_msgs->header.stamp;
    lsr1.header.frame_id = laser1_msgs->header.frame_id;
    lsr1.angle_min = laser1_msgs->angle_min;
    lsr1.angle_max = laser1_msgs->angle_max;
    lsr1.angle_increment = laser1_msgs->angle_increment;
    lsr1.time_increment = laser1_msgs->time_increment;
    lsr1.range_min = 0.10;
    lsr1.range_max = 30.0;
    lsr1.ranges.resize(ranges);
    for(int i = 0; i < ranges; i++)
    {
        lsr1.ranges[i] = laser1_msgs->ranges[i];
    }
}

void StepServer::cb_las_2(const sensor_msgs::LaserScanPtr & laser2_msgs)
{
    //cout<<"get the laser data 2"<<endl;
    ranges = laser2_msgs->ranges.size();
    lsr2.header.stamp = laser2_msgs->header.stamp;
    lsr2.header.frame_id = laser2_msgs->header.frame_id;
    lsr2.angle_min = laser2_msgs->angle_min;
    lsr2.angle_max = laser2_msgs->angle_max;
    lsr2.angle_increment = laser2_msgs->angle_increment;
    lsr2.time_increment = laser2_msgs->time_increment;
    lsr2.range_min = 0.10;
    lsr2.range_max = 30.0;
    lsr2.ranges.resize(ranges);
    for(int i = 0; i < ranges; i++)
    {
        lsr2.ranges[i] = laser2_msgs->ranges[i];
    }
}

geometry_msgs::Pose StepServer::TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose)
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


bool StepServer::step(mecanum_sim::DRLStep::Request &req, mecanum_sim::DRLStep::Response &res)
{
  ros::Publisher fl_pub = nh.advertise<std_msgs::Float32>("front_left",1,true);
  ros::Publisher fr_pub = nh.advertise<std_msgs::Float32>("front_right",1,true);
  ros::Publisher bl_pub = nh.advertise<std_msgs::Float32>("back_left",1,true);
  ros::Publisher br_pub = nh.advertise<std_msgs::Float32>("back_right",1,true);
  ros::Rate loop_rate(20);
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
//  ros::waitForShutdown();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;
  cout<<"hello client, this is server!"<<endl;

  ros::Subscriber sub_joint=nh.subscribe("/gazebo/link_states", 10, &StepServer::cb_pr, this);
  int r=5;
  while(r)
  {
  ros::spinOnce();
  loop_rate.sleep();
  r--;
  }


  int ind=(num_steps%100)/20;
  int step;
  double nx,ny;
  double dist;
  double r_near = 0.6;
  double yita = 0.707;
  double x_tar = 0;
  double y_tar = 0.1;
  double x_tar_ind;
  double y_tar_ind;
  cout<<"ind is :"<<ind<<endl;

  if((num_steps%20)==0)
  {
      switch(ind)
      {
          case 0:
              d_1 = sqrt((x_tar-r_near-pa_prior.people1_pose.position.x)*(x_tar-r_near-pa_prior.people1_pose.position.x)+(y_tar+0.0-pa_prior.people1_pose.position.y)*(y_tar+0.0-pa_prior.people1_pose.position.y));
              nx1 = (x_tar-r_near-pa_prior.people1_pose.position.x)/d_1;
              ny1 = (y_tar+0.0-pa_prior.people1_pose.position.y)/d_1;
              x_tar_ind = x_tar - r_near;
              y_tar_ind = y_tar;
          break;
          case 1:
              d_2 = sqrt((x_tar-r_near*yita-pa_prior.people2_pose.position.x)*(x_tar-r_near*yita-pa_prior.people2_pose.position.x)+(y_tar-r_near*yita-pa_prior.people2_pose.position.y)*(y_tar-r_near*yita-pa_prior.people2_pose.position.y));
              nx2 = (x_tar-r_near*yita-pa_prior.people2_pose.position.x)/d_2;
              ny2 = (y_tar-r_near*yita-pa_prior.people2_pose.position.y)/d_2;
              x_tar_ind = x_tar - r_near*yita;
              y_tar_ind = y_tar - r_near*yita;
          break;
          case 2:
              d_3 = sqrt((x_tar+0.0-pa_prior.people3_pose.position.x)*(x_tar+0.0-pa_prior.people3_pose.position.x)+(y_tar-r_near-pa_prior.people3_pose.position.y)*(y_tar-r_near-pa_prior.people3_pose.position.y));
              nx3 = (x_tar+0.0-pa_prior.people3_pose.position.x)/d_3;
              ny3 = (y_tar-r_near-pa_prior.people3_pose.position.y)/d_3;
              x_tar_ind = x_tar;
              y_tar_ind = y_tar - r_near;
          break;
          case 3:
              d_4 = sqrt((x_tar+r_near*yita-pa_prior.people4_pose.position.x)*(x_tar+r_near*yita-pa_prior.people4_pose.position.x)+(y_tar-r_near*yita-pa_prior.people4_pose.position.y)*(y_tar-r_near*yita-pa_prior.people4_pose.position.y));
              nx4 = (x_tar+r_near*yita-pa_prior.people4_pose.position.x)/d_4;
              ny4 = (y_tar-r_near*yita-pa_prior.people4_pose.position.y)/d_4;
              x_tar_ind = x_tar + r_near*yita;
              y_tar_ind = y_tar - r_near*yita;
          break;
          case 4:
              d_5 = sqrt((x_tar+r_near-pa_prior.people5_pose.position.x)*(x_tar+r_near-pa_prior.people5_pose.position.x)+(y_tar+0.0-pa_prior.people5_pose.position.y)*(y_tar+0.0-pa_prior.people5_pose.position.y));
              nx5 = (x_tar+r_near-pa_prior.people5_pose.position.x)/d_5;
              ny5 = (y_tar+0.0-pa_prior.people5_pose.position.y)/d_5;
              x_tar_ind = x_tar + r_near;
              y_tar_ind = y_tar + 0.0;
          break;
      }
  }

  switch(ind)
  {
      case 0:
          modelstate.model_name = "unit_cylinder_1";
          modelstate.reference_frame = "world";
          nx=nx1;
          ny=ny1;
          dist=d_1;
          cout<<"now move the people 1 ->"<<endl;
      break;
      case 1:
          modelstate.model_name = "unit_cylinder_2";
          modelstate.reference_frame = "world";
          nx=nx2;
          ny=ny2;
          dist=d_2;
          cout<<"now move the people 2 ->"<<endl;
      break;
      case 2:
          modelstate.model_name = "unit_cylinder_3";
          modelstate.reference_frame = "world";
          nx=nx3;
          ny=ny3;
          dist=d_3;
          cout<<"now move the people 3 ->"<<endl;
      break;
      case 3:
          modelstate.model_name = "unit_cylinder_4";
          modelstate.reference_frame = "world";
          nx=nx4;
          ny=ny4;
          dist=d_4;
          cout<<"now move the people 4 ->"<<endl;
      break;
      case 4:
          modelstate.model_name = "unit_cylinder_5";
          modelstate.reference_frame = "world";
          nx=nx5;
          ny=ny5;
          dist=d_5;
          cout<<"now move the people 5 ->"<<endl;
      break;
  }

  cout<<"nx is :"<<nx<<","<<"ny is :"<<ny<<endl;
  double vx,vy;
  double pose_x,pose_y;
  step=(num_steps%100)-20*ind;
  cout<<"********step is :"<<step<<endl;
  double v_0 = dist/55;
  if((step>=0)&&(step<=9))
  {
      vx=v_0*(10-step)*nx;
      vy=v_0*(10-step)*ny;
      pose_x = x_tar_ind - dist*(1-(20-step)*(step+1)/110.0+(10-step)/55.0)*nx;
      pose_y = y_tar_ind - dist*(1-(20-step)*(step+1)/110.0+(10-step)/55.0)*ny;
  }
  else if((step>=10)&&(step<=19))
  {
      vx=-v_0*(step-9)*nx;
      vy=-v_0*(step-9)*ny;
      pose_x = x_tar_ind - dist*((step-8)*(step-9)/110.0-(step-9)/55.0)*nx;
      pose_y = y_tar_ind - dist*((step-8)*(step-9)/110.0-(step-9)/55.0)*ny;
  }
  cout<<"vx is :"<<vx<<","<<"vy is :"<<vy<<endl;

  //設定cylinder Position
     geometry_msgs::Point cyl_position;
     cyl_position.x = pose_x;
     cyl_position.y = pose_y;
     cyl_position.z = 0.0;
     //設定cylinder orientation
     geometry_msgs::Quaternion cyl_orientation;
     cyl_orientation.x = 0.0;
     cyl_orientation.y = 0.0;
     cyl_orientation.z = 0.0;
     cyl_orientation.w = 1.0;

     geometry_msgs::Pose cyl_pose;
     cyl_pose.position = cyl_position;
     cyl_pose.orientation = cyl_orientation;

  geometry_msgs::Twist model_twist;
  model_twist.linear.x = vx;
  model_twist.linear.y = vy;
  model_twist.linear.z = 0.0;
  model_twist.angular.x = 0.0;
  model_twist.angular.y = 0.0;
  model_twist.angular.z = 0.0;

  modelstate.twist = model_twist;

  modelstate.pose = cyl_pose;

  setmodelstate.request.model_state = modelstate;
  int c=0;

  /*client.call(setmodelstate);
  ros::Duration(2.0).sleep();*///two way the same effection;
  double v_fl=req.action[0]+req.action[1]-req.action[2]*(TRACK+WHEELBASE)/2;
  double v_fr=-req.action[0]+req.action[1]+req.action[2]*(TRACK+WHEELBASE)/2;
  double v_bl=-req.action[0]+req.action[1]-req.action[2]*(TRACK+WHEELBASE)/2;
  double v_br=req.action[0]+req.action[1]+req.action[2]*(TRACK+WHEELBASE)/2;
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

  cout<<"begin calculate the theoretical position of the future robot!"<<endl;
  pa_theory=pa_prior;
  switch(ind)
  {
      case 0:
          pa_theory.people1_pose.position.x=pa_prior.people1_pose.position.x+vx;
          pa_theory.people1_pose.position.y=pa_prior.people1_pose.position.y+vy;
      case 1:
          pa_theory.people2_pose.position.x=pa_prior.people2_pose.position.x+vx;
          pa_theory.people2_pose.position.y=pa_prior.people2_pose.position.y+vy;
      case 2:
          pa_theory.people3_pose.position.x=pa_prior.people3_pose.position.x+vx;
          pa_theory.people3_pose.position.y=pa_prior.people3_pose.position.y+vy;
      case 3:
          pa_theory.people4_pose.position.x=pa_prior.people4_pose.position.x+vx;
          pa_theory.people4_pose.position.y=pa_prior.people4_pose.position.y+vy;
      case 4:
          pa_theory.people5_pose.position.x=pa_prior.people5_pose.position.x+vx;
          pa_theory.people5_pose.position.y=pa_prior.people5_pose.position.y+vy;
  }
  pa_theory.robot_pose.position.x=pa_prior.robot_pose.position.x+req.action[0];
  pa_theory.robot_pose.position.y=pa_prior.robot_pose.position.y+req.action[1];
  pa_theory.robot_pose.position.z=pa_prior.robot_pose.position.z;
  double theta=2*acos(pa_prior.robot_pose.orientation.w);
  cout<<"pa_prior.robot_pose.orientation.w :"<<pa_prior.robot_pose.orientation.w<<endl;
  double theta_=theta+req.action[2];
  cout<<"theta future is :"<<theta<<endl;
  pa_theory.robot_pose.orientation.x=0;
  pa_theory.robot_pose.orientation.y=0;
  pa_theory.robot_pose.orientation.z=sin(theta_/2.0);
  pa_theory.robot_pose.orientation.w=cos(theta_/2.0);
  pa_theory.calcRelativePosition();

  std::vector<double> joints;
//  if((num_steps%50)==0)
//  {
    joints.push_back(0);
    joints.push_back(-2.83);
    joints.push_back(2.76);
    joints.push_back(-1.59);
    joints.push_back(0);
    joints.push_back(0);
//    cout<<"start robot move up!"<<endl;
//    //bool suc_arm=create_motion_plan(joints);
//    moveit::planning_interface::MoveGroupInterface::Plan plan;
//    moveit_msgs::RobotState robot_state;
//    cout<<"hello world 1"<<endl;
//    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
//    robot_state::RobotState current_robot_state = current_scene->getCurrentState();

//    cout<<"hello world 2"<<endl;
//    robot_arm->setNumPlanningAttempts(4);
//    robot_arm->setPlannerId("RRTConnectkConfigDefault");
//    robot_arm->setStartState(current_robot_state);
//    robot_arm->setJointValueTarget(joints);
//    bool success = false;
//    success = (robot_arm->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    cout<<"hello world 3"<<endl;
//    robot_arm->execute(plan);
//    cout<<"hello world 4"<<endl;
//  }

  geometry_msgs::Pose obj_pose_2 = obj_pose;
  obj_pose_2.orientation.w = cos(req.action[3]/2.0);
  obj_pose_2.orientation.x = sin(req.action[3]/2.0);


  geometry_msgs::Pose obj_pose_rel=TransformFromObjToRobot(obj_pose_2,pa_theory.robot_pose);
  cout<<"relative pose of object to robot :"<<obj_pose_rel<<endl;
  planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
  robot_state::RobotState current_robot_state = current_scene->getCurrentState();
  robot_arm->setStartState(current_robot_state);
  vector<double> forward_joints;
  vector<double> back_joints;
  cout<<"Add people and desk to the rViz!"<<endl;
  //------------------------desk---------------------------//
  moveit_msgs::CollisionObject box;
  box.header.frame_id = "base_footprint";
  box.id = "box";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 5;
  primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 0.7;

  geometry_msgs::Pose pose=pa_theory.desk_pose_rel;
  cout<<"The relative pose of desk to the robot is :"<<pose<<endl;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(pose);
  box.operation = box.ADD;
  //--------------------------------------------------------//

  //------------------------people1---------------------------//
  moveit_msgs::CollisionObject cyl1;
  cyl1.header.frame_id = "base_footprint";
  cyl1.id = "cylinder1";
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.CYLINDER;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 1.8;
  primitive1.dimensions[1] = 0.24;

  geometry_msgs::Pose pose1=pa_theory.people1_pose_rel;

  cyl1.primitives.push_back(primitive1);
  cyl1.primitive_poses.push_back(pose1);
  cyl1.operation = cyl1.ADD;
  //--------------------------------------------------------//

  //------------------------people2---------------------------//
  moveit_msgs::CollisionObject cyl2;
  cyl2.header.frame_id = "base_footprint";
  cyl2.id = "cylinder2";
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive2.CYLINDER;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 1.8;
  primitive2.dimensions[1] = 0.24;

  geometry_msgs::Pose pose2=pa_theory.people2_pose_rel;

  cyl2.primitives.push_back(primitive2);
  cyl2.primitive_poses.push_back(pose2);
  cyl2.operation = cyl2.ADD;
  //--------------------------------------------------------//

  //------------------------people3---------------------------//
  moveit_msgs::CollisionObject cyl3;
  cyl3.header.frame_id = "base_footprint";
  cyl3.id = "cylinder3";
  shape_msgs::SolidPrimitive primitive3;
  primitive3.type = primitive3.CYLINDER;
  primitive3.dimensions.resize(3);
  primitive3.dimensions[0] = 1.8;
  primitive3.dimensions[1] = 0.24;

  geometry_msgs::Pose pose3=pa_theory.people3_pose_rel;

  cyl3.primitives.push_back(primitive3);
  cyl3.primitive_poses.push_back(pose3);
  cyl3.operation = cyl3.ADD;
  //--------------------------------------------------------//

  //------------------------people4---------------------------//
  moveit_msgs::CollisionObject cyl4;
  cyl4.header.frame_id = "base_footprint";
  cyl4.id = "cylinder4";
  shape_msgs::SolidPrimitive primitive4;
  primitive4.type = primitive4.CYLINDER;
  primitive4.dimensions.resize(3);
  primitive4.dimensions[0] = 1.8;
  primitive4.dimensions[1] = 0.24;

  geometry_msgs::Pose pose4=pa_theory.people4_pose_rel;

  cyl4.primitives.push_back(primitive4);
  cyl4.primitive_poses.push_back(pose4);
  cyl4.operation = cyl4.ADD;
  //--------------------------------------------------------//

  //------------------------people5---------------------------//
  moveit_msgs::CollisionObject cyl5;
  cyl5.header.frame_id = "base_footprint";
  cyl5.id = "cylinder5";
  shape_msgs::SolidPrimitive primitive5;
  primitive5.type = primitive5.CYLINDER;
  primitive5.dimensions.resize(3);
  primitive5.dimensions[0] = 1.8;
  primitive5.dimensions[1] = 0.24;

  geometry_msgs::Pose pose5=pa_theory.people5_pose_rel;

  cyl5.primitives.push_back(primitive5);
  cyl5.primitive_poses.push_back(pose5);
  cyl5.operation = cyl5.ADD;
  //--------------------------------------------------------//


  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(box);
  collision_objects.push_back(cyl1);
  collision_objects.push_back(cyl2);
  collision_objects.push_back(cyl3);
  collision_objects.push_back(cyl4);
  collision_objects.push_back(cyl5);
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(box);
  planning_scene.world.collision_objects.push_back(cyl1);
  planning_scene.world.collision_objects.push_back(cyl2);
  planning_scene.world.collision_objects.push_back(cyl3);
  planning_scene.world.collision_objects.push_back(cyl4);
  planning_scene.world.collision_objects.push_back(cyl5);
  planning_scene.is_diff = true;
  planning_scene_interface.addCollisionObjects(collision_objects);
  planning_scene_diff_publisher.publish(planning_scene);

  string word[6]={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
  std::vector<std::string> joint_names_0(word,word+6);
  // //find IK solutions
  current_scene = scene_->getPlanningScene();
  current_robot_state = current_scene->getCurrentState();
  robot_arm->setStartState(current_robot_state);

   const robot_state::JointModelGroup *joint_model_group = current_robot_state.getJointModelGroup(robot_arm->getName());

   bool successIK = current_robot_state.setFromIK(joint_model_group, obj_pose_rel);
   cout<<"find grasp pose:set from ik is success?:"<<successIK<<endl;
   res.reward = 0;
   if(successIK)
   {
      res.reward += 10;
      std::vector<double> joint_values;
      current_robot_state.copyJointGroupPositions(joint_model_group, joint_values);
      const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
      for(std::size_t i = 0; i < joint_names.size(); ++i)
      {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

      back_joints.clear();

      back_joints.push_back(joints[0]);
      back_joints.push_back(joints[1]);
      back_joints.push_back(joints[2]);
      back_joints.push_back(joints[3]);
      back_joints.push_back(joints[4]);
      back_joints.push_back(joints[5]);

      forward_joints.clear();

      forward_joints.push_back(joint_values[0]);
      forward_joints.push_back(joint_values[1]);
      forward_joints.push_back(joint_values[2]);
      forward_joints.push_back(joint_values[3]);
      forward_joints.push_back(joint_values[4]);
      forward_joints.push_back(joint_values[5]);

      current_robot_state.copyJointGroupPositions( robot_arm->getName(), back_joints);
      current_robot_state.setVariablePositions( joint_names_0, forward_joints);
      bool Is_Colliding = current_scene->isStateColliding( current_robot_state );
      if(Is_Colliding)
      {
          cout<<"Although the IK is succeed, the robot is in colliding."<<endl;
          res.reward -= 8;
      }
   }
   else
   {
       res.reward += 0;
       back_joints.clear();

       back_joints.push_back(joints[0]);
       back_joints.push_back(joints[1]);
       back_joints.push_back(joints[2]);
       back_joints.push_back(joints[3]);
       back_joints.push_back(joints[4]);
       back_joints.push_back(joints[5]);

       forward_joints.clear();

       forward_joints.push_back(joints[0]);
       forward_joints.push_back(joints[1]);
       forward_joints.push_back(joints[2]);
       forward_joints.push_back(joints[3]);
       forward_joints.push_back(joints[4]);
       forward_joints.push_back(joints[5]);

       current_robot_state.copyJointGroupPositions( robot_arm->getName(), back_joints);
       current_robot_state.setVariablePositions( joint_names_0, forward_joints);
       bool Is_Colliding = current_scene->isStateColliding( current_robot_state );
       if(Is_Colliding)
       {
           cout<<"The IK is failed, and the robot is in colliding."<<endl;
           res.reward -= 8;
       }
   }
   if((pa_theory.robot_pose.position.x<-8.0)||(pa_theory.robot_pose.position.x>8.0)||(pa_theory.robot_pose.position.y<-5.6)||(pa_theory.robot_pose.position.x>2.9))
   {
       res.reward -= 5;
   }

   //从世界中移除物体
   ROS_INFO("Remove the object from the world");
   std::vector<std::string> object_ids;
   object_ids.push_back(box.id);
   object_ids.push_back(cyl1.id);
   object_ids.push_back(cyl2.id);
   object_ids.push_back(cyl3.id);
   object_ids.push_back(cyl4.id);
   object_ids.push_back(cyl5.id);
   planning_scene_interface.removeCollisionObjects(object_ids);

//  while(c<=20)
//  {
      cout<<"Lets real move"<<endl;
      fl_pub.publish(vfl);
      fr_pub.publish(vfr);
      bl_pub.publish(vbl);
      br_pub.publish(vbr);//move robot
      client.call(setmodelstate);//move people
      ros::Duration(1.0).sleep();
//      ROS_INFO("%f, %f",modelstate.pose.position.x,modelstate.pose.position.y);
//      ros::spinOnce();
//      loop_rate.sleep();
//      c++;
  //}


      double pose_x_aft,pose_y_aft;

      if((step>=0)&&(step<=9))
      {
          pose_x_aft = x_tar_ind - dist*(1-(20-step)*(step+1)/110.0)*nx;
          pose_y_aft = y_tar_ind - dist*(1-(20-step)*(step+1)/110.0)*ny;
      }
      else if((step>=10)&&(step<=19))
      {
          pose_x_aft = x_tar_ind - dist*((step-8)*(step-9)/110.0)*nx;
          pose_y_aft = y_tar_ind - dist*((step-8)*(step-9)/110.0)*ny;
      }

      //設定cylinder Position
         cyl_position.x = pose_x_aft;
         cyl_position.y = pose_y_aft;
         cyl_position.z = 0.0;
         //設定cylinder orientation
         cyl_orientation.x = 0.0;
         cyl_orientation.y = 0.0;
         cyl_orientation.z = 0.0;
         cyl_orientation.w = 1.0;

         geometry_msgs::Pose cyl_pose_aft;
         cyl_pose_aft.position = cyl_position;
         cyl_pose_aft.orientation = cyl_orientation;

  model_twist.linear.x = 0.0;
  model_twist.linear.y = 0.0;
  model_twist.linear.z = 0.0;
  model_twist.angular.x = 0.0;
  model_twist.angular.y = 0.0;
  model_twist.angular.z = 0.0;
  modelstate.pose = cyl_pose_aft;
  modelstate.twist = model_twist;

  setmodelstate.request.model_state = modelstate;
  vfl.data = 0;
  vfr.data = 0;
  vbl.data = 0;
  vbr.data = 0;

  client.call(setmodelstate);//move people
  fl_pub.publish(vfl);
  fr_pub.publish(vfr);
  bl_pub.publish(vbl);
  br_pub.publish(vbr);//move robot
  ros::Duration(1.0).sleep();

  ros::Subscriber sub_joint2=nh.subscribe("/gazebo/link_states", 10, &StepServer::cb_af, this);
  int r2=5;
  while(r2--)
  {
  ros::spinOnce();
  loop_rate.sleep();
  }

  ros::Subscriber sub_laser1=nh.subscribe("/laser/scan1", 5, &StepServer::cb_las_1, this);
  int r3=5;
  while(r3--)
  {
  ros::spinOnce();
  loop_rate.sleep();
  }

  ros::Subscriber sub_laser2=nh.subscribe("/laser/scan2", 5, &StepServer::cb_las_2, this);
  int r4=5;
  while(r4--)
  {
  ros::spinOnce();
  loop_rate.sleep();
  }

  cout<<"get the laser data over"<<endl;
  res.state.resize(1080);
  for(int i = 0; i < ranges; i ++)
  {
      res.state[i]=lsr1.ranges[i];
  }
  for(int i = ranges; i < 2*ranges; i ++)
  {
      res.state[i]=lsr2.ranges[i-ranges];
  }

  geometry_msgs::Pose obj_pose_rel_after=TransformFromObjToRobot(obj_pose,pa_after.robot_pose);
  res.object.resize(2);
  res.object[0] = obj_pose_rel_after.position.x;
  res.object[1] = obj_pose_rel_after.position.y;
  G = 3.8997;
  res.reward = res.reward + G/sqrt((res.object[0]-0.2)*(res.object[0]-0.2)+res.object[1]*res.object[1]);

  res.position.resize(4);
  res.position[0] = pa_after.robot_pose.position.x;
  res.position[1] = pa_after.robot_pose.position.y;
  res.position[2] = 2*acos(pa_after.robot_pose.orientation.w);
  res.position[3] = req.action[3];

  omega1 = 2.0;
  omega2 = 2.0;
  if(num_steps==0)
  {
      delta_grasp_angle = res.position[3];
  }
  else
  {
      delta_grasp_angle = res.position[3] - pa_grasp[pa_grasp.size()-1];
  }

  pa_grasp.push_back(res.position[3]);
  res.reward = res.reward-omega1*abs(req.action[2])-omega2*abs(delta_grasp_angle);
  cout<<"Reward of this step is :"<<res.reward<<endl;
  num_steps++;
  cout<<"number of step :"<<num_steps<<endl;
  return true;
}


int main (int argc, char **argv)
{
  ros::init(argc,argv,"step_server");

  //-------------init nh-----------//

  ros::NodeHandle n;

  //-------------init jnt_to_pose_solver-----------//

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

  scene->startStateMonitor();
  scene->startSceneMonitor();
  scene->startWorldGeometryMonitor();

  //------------init robot------------//

  moveit::planning_interface::MoveGroup *robot = new moveit::planning_interface::MoveGroup("manipulator");


  //-----------start ros service-------------//

  StepServer ss(n,scene,robot,jnt_to_pose_solver);

  return 0;
}
