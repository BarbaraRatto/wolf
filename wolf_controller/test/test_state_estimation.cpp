// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// OpenSoT
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/utils/Affine.h>

#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/tasks/velocity/Postural.h>

// WoLF
#include <wolf_controller/ContactForces.h>

#include "test_common_utils.h"

#define N_JOINTS 18

using namespace wolf_controller;

static Eigen::VectorXd _q_meas(N_JOINTS);
static Eigen::VectorXd _qdot_meas(N_JOINTS);
static Eigen::VectorXd _q(N_JOINTS);
static Eigen::VectorXd _qdot(N_JOINTS);
static Eigen::VectorXd _fbq(FLOATING_BASE_DOFS);
static Eigen::VectorXd _fbqdot(FLOATING_BASE_DOFS);

static QuadrupedRobot::Ptr _robot;
static OpenSoT::AutoStack::Ptr _stack;
static std::unique_ptr<OpenSoT::solvers::iHQP> _solver;
static std::map<std::string,OpenSoT::tasks::velocity::Cartesian::Ptr> _contact_tasks;
static OpenSoT::tasks::velocity::Postural::Ptr _postural;
static OpenSoT::tasks::velocity::Cartesian::Ptr _imu_task;
static OpenSoT::tasks::Aggregated::Ptr _aggregated_contacts;
static std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::Imu,ContactForces>> _state_sub;
static double _t = 0.0;
static double _t_prev = 0.0;
static bool _init = true;

void init(Eigen::VectorXd& q_init, Eigen::VectorXd& qdot_init)
{
  // initialize the q
  _q_meas = _q = q_init;
  _qdot_meas = _qdot = qdot_init;

  // update robot's state
  _robot->setJointPosition(_q);
  _robot->setJointVelocity(_qdot);
  _robot->update();

  // define indices for the tasks
  std::list<unsigned int> id_joints;
  id_joints.resize(12);
  std::list<unsigned int>::iterator it;
  unsigned int idx = FLOATING_BASE_DOFS;
  for (it = id_joints.begin(); it != id_joints.end(); ++it)
  {
    *it = idx;
    idx++;
  }
  std::list<unsigned int> id_XYZ   = {0,1,2}; //xyz
  std::list<unsigned int> id_RPY   = {3,4,5}; //r,p,y

  // contact tasks
  auto contact_links = _robot->getFootNames();
  for(unsigned int i = 0; i < contact_links.size(); ++i)
  {
    _contact_tasks[contact_links[i]] = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(contact_links[i],_q,*_robot,contact_links[i],WORLD_FRAME_NAME);
    _contact_tasks[contact_links[i]]->setLambda(0.1);
  }
  _aggregated_contacts = std::make_shared<OpenSoT::tasks::Aggregated>(_contact_tasks[contact_links[0]]%id_XYZ,_q.size());
  for(unsigned int i=1;i<contact_links.size();i++)
    _aggregated_contacts = _aggregated_contacts + _contact_tasks[contact_links[i]]%id_XYZ;

  // imu task
  _imu_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(_robot->getImuSensorName(),_q,*_robot,_robot->getImuSensorName(),WORLD_FRAME_NAME);
  _imu_task->setLambda(10.0);

  // postural task
  _postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
  _postural->setLambda(0.0);
  Eigen::MatrixXd w(N_JOINTS,N_JOINTS);
  w.setIdentity();
  w.block(0,0,6,6).setZero();
  _postural->setWeight(w);

  // define the stack
  //_stack /= _aggregated_contacts + _imu_task%id_RPY << _postural;
  _stack /= _aggregated_contacts + _imu_task%id_RPY << _postural;
  _stack->update(_q);

  // create the solver
  _solver = std::make_unique<OpenSoT::solvers::iHQP>(_stack->getStack(),_stack->getBounds(),1e6);
  _solver->setSolverID("FloatingBaseEstimation");
}

void update(const sensor_msgs::JointState::ConstPtr& joints_msg,
            const sensor_msgs::Imu::ConstPtr& imu_msg,
            const ContactForces::ConstPtr& cf_msg)
{

  _t = ros::Time::now().toSec();

  //double dt = _t - _t_prev;

  double dt = 0.001;

  if(dt > 0.0)
  {
    // Get the measured q and qdot
    for(unsigned int i =0;i<joints_msg->name.size();i++)
    {
      _q_meas(i+FLOATING_BASE_DOFS) = joints_msg->position[i];
      _qdot_meas(i+FLOATING_BASE_DOFS) = joints_msg->velocity[i];
    }

    // Initialize
    if(_init)
    {
      init(_q_meas,_qdot_meas);
      _init = false;
    }

    // Update imu state
    Eigen::Quaterniond imu_quat;
    Eigen::Vector6d imu_vel_ref;
    Eigen::Affine3d imu_pose;
    Eigen::Vector3d imu_rpy;
    Eigen::Matrix3d imu_R;
    imu_quat.w() = imu_msg->orientation.w;
    imu_quat.x() = imu_msg->orientation.x;
    imu_quat.y() = imu_msg->orientation.y;
    imu_quat.z() = imu_msg->orientation.z;
    quatToRot(imu_quat.normalized(),imu_R);
    imu_pose.setIdentity();
    imu_pose.linear() = imu_R.transpose();
    imu_vel_ref << 0., 0., 0., imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    //_imu_task->setReference(imu_pose,imu_vel_ref);
    _imu_task->setReference(imu_pose);

    // set joint velocities to postural task
    _postural->setReference(_q_meas,_qdot_meas * dt);

    // set contact state
    for(unsigned int i=0; i<cf_msg->name.size(); i++)
    {
      //if(cf_msg->contact[i])
      //  ROS_INFO_STREAM("Active contact: "<< cf_msg->name[i]);
      _contact_tasks[cf_msg->name[i]]->setActive(cf_msg->contact[i]);
    }

    // Update robot's joint states
    _robot->setJointPosition(_q);
    _robot->setJointVelocity(_qdot);
    _robot->update();

    // Solve ik
    _stack->update(_q);
    if(_solver->solve(_qdot))
    {

      // integrate solution
      _q += _qdot * dt;

      _fbqdot = _qdot.segment(0,6);
      _fbq = _q.segment(0,6);

      //ROS_INFO_STREAM("DT: "<<dt);
      ROS_INFO_STREAM("q: "<<_q.transpose());
      ROS_INFO_STREAM("q_meas: "<< _q_meas.transpose());
      ROS_INFO_STREAM("qdot: "<<_qdot.transpose());
      ROS_INFO_STREAM("qdot_meas: "<< _qdot_meas.transpose());
      ROS_INFO_STREAM("FB pos: "<<_fbq.transpose());
      ROS_INFO_STREAM("FB vel: "<<_fbqdot.transpose());
      rotToRpy(imu_R,imu_rpy);
      ROS_INFO_STREAM("imu_rpy: "<< imu_rpy.transpose());

      // Update FB
      //Eigen::Affine3d fb_pose;
      //Eigen::Matrix3d fb_R;
      //fb_pose.translation() << _fbq.segment(0,3);
      //rpyToRot(_fbq.segment(3,3),fb_R);
      //fb_pose.linear() << fb_R;
      //_robot->setFloatingBaseState(fb_pose,_fbqdot);
      //_robot->update();
    }
    else
      ROS_WARN("Can not solve!");
  }
  _t_prev = _t;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_state_estimation");
  ros::NodeHandle root_nh;

  // reset
  _q_meas.fill(0.0);
  _qdot_meas.fill(0.0);
  _q.fill(0.0);
  _qdot.fill(0.0);
  _fbq.fill(0.0);
  _fbqdot.fill(0.0);

  _robot.reset(createRobotModel(root_nh));

  message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
  message_filters::Subscriber<ContactForces> cf_sub;

  joint_state_sub.subscribe(root_nh,_robot->getRobotName()+"/joint_states",100);
  imu_sub.subscribe(root_nh,"/trunk_imu/data",100);
  cf_sub.subscribe(root_nh,_robot->getRobotName()+"/wolf_controller/contact_forces",100);
  _state_sub = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::Imu,ContactForces> >(joint_state_sub,imu_sub,cf_sub,100);
  _state_sub->registerCallback(update);

  //ros::Subscriber sub = root_nh.subscribe(_robot->getRobotName()+"/joint_states", 1000, update);

  ros::spin();

  return 0;
}
