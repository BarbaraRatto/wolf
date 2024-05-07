// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

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
#include <wolf_msgs/ContactForces.h>

// Rt logger
#include <rt_logger/rt_logger.h>

// Test utils goodies
#include "test_common_utils.h"

#define N_JOINTS 18

static Eigen::VectorXd _q_meas(N_JOINTS);
static Eigen::VectorXd _qdot_meas(N_JOINTS);
static Eigen::VectorXd _q(N_JOINTS);
static Eigen::VectorXd _qdot(N_JOINTS);
static Eigen::VectorXd _postural_error(N_JOINTS);
static Eigen::VectorXd _postural_reference(N_JOINTS);
static Eigen::VectorXd _postural_reference_dot(N_JOINTS);

static wolf_controller::QuadrupedRobot::Ptr _robot;
static OpenSoT::AutoStack::Ptr _stack;
static std::unique_ptr<OpenSoT::solvers::iHQP> _solver;
static std::map<std::string,OpenSoT::tasks::velocity::Cartesian::Ptr> _contact_tasks;
static OpenSoT::tasks::velocity::Postural::Ptr _postural;
static OpenSoT::tasks::velocity::Cartesian::Ptr _imu_task;
static OpenSoT::tasks::Aggregated::Ptr _aggregated_contacts;
static std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::Imu,wolf_msgs::ContactForces>> _state_sub;
static double _t = 0.0;
static double _t_prev = 0.0;
static bool _initialize = true;

static Eigen::Quaterniond _imu_quat;
static Eigen::Vector6d _imu_vel_ref;
static Eigen::Affine3d _imu_pose;
static Eigen::Vector3d _imu_rpy;
static Eigen::Matrix3d _imu_R;

static Eigen::Affine3d _fb_pose;
static Eigen::Vector3d _fb_rpy;
static Eigen::Vector3d _fb_xyz;
static Eigen::Vector6d _fbq;
static Eigen::Vector6d _fbqdot;
static Eigen::Vector3d _contact_force;

static nav_msgs::Odometry _odom_msg;
static ros::Publisher _odom_pub;

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
    _contact_tasks[contact_links[i]]->setLambda(0.0);
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
  //Eigen::MatrixXd w(N_JOINTS,N_JOINTS);
  //w.setIdentity() * 10.0;
  //w.block(0,0,6,6).setZero();
  //_postural->setWeight(w);

  // define the stack
  _stack /= _aggregated_contacts + _imu_task%id_RPY << _postural%id_joints;
  _stack->update(_q);

  // create the solver
  _solver = std::make_unique<OpenSoT::solvers::iHQP>(_stack->getStack(),_stack->getBounds(),1e6);
  _solver->setSolverID("FloatingBaseEstimation");
}

void update(const sensor_msgs::JointState::ConstPtr& joints_msg,
            const sensor_msgs::Imu::ConstPtr& imu_msg,
            const wolf_msgs::ContactForces::ConstPtr& cf_msg)
{

  _t = ros::Time::now().toSec();

  double dt = _t - _t_prev;

  if(dt > 0.0)
  {
    // Get the measured q and qdot
    for(unsigned int i =0;i<joints_msg->name.size();i++)
    {
      _q_meas(i+FLOATING_BASE_DOFS) = joints_msg->position[i];
      _qdot_meas(i+FLOATING_BASE_DOFS) = joints_msg->velocity[i];
    }

    // Initialize
    if(_initialize)
    {
      init(_q_meas,_qdot_meas);
      _initialize = false;
    }

    // Update imu state
    _imu_quat.w() = imu_msg->orientation.w;
    _imu_quat.x() = imu_msg->orientation.x;
    _imu_quat.y() = imu_msg->orientation.y;
    _imu_quat.z() = imu_msg->orientation.z;
    wolf_controller_utils::quatToRot(_imu_quat.normalized(),_imu_R);
    _imu_pose.translation().setZero();
    _imu_pose.linear() = _imu_R;
    _imu_vel_ref << 0., 0., 0., imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    //_imu_task->setReference(imu_pose,imu_vel_ref); // TODO
    _imu_task->setReference(_imu_pose);

    // set joint velocities to postural task
    _postural->setReference(_q_meas,_qdot_meas);
    _postural_error = _postural->getError();
    _postural->getReference(_postural_reference,_postural_reference_dot);

    // set contact state
    for(unsigned int i=0; i<cf_msg->name.size(); i++)
    {
      //if(cf_msg->contact[i])
      //  ROS_INFO_STREAM("Active contact: "<< cf_msg->name[i]);
      //_contact_tasks[cf_msg->name[i]]->setActive(cf_msg->contact[i]);
      _contact_force.x() = cf_msg->contact_forces[i].force.x;
      _contact_force.y() = cf_msg->contact_forces[i].force.y;
      _contact_force.z() = cf_msg->contact_forces[i].force.z;
      _contact_tasks[cf_msg->name[i]]->setWeight(_contact_force.norm());
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
      _q = _qdot * dt + _q;

      // Get FB pose
      _robot->getFloatingBasePose(_fb_pose);
      _robot->getFloatingBaseTwist(_fbqdot);
      wolf_controller_utils::rotToRpy(_fb_pose.linear(),_fb_rpy);
      _fb_xyz = _fb_pose.translation();
      _fbq << _fb_xyz, _fb_rpy;

    }
    else
      ROS_WARN("Can not solve!");

    // Publish odometry
    // The nav_msgs/Odometry message stores an estimate of the position and velocity of a robot in free space:
    // This represents an estimate of a position and velocity in free space.
    // The pose in this message should be specified in the coordinate frame given by header.frame_id (in our case odom)
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id (in our case the basefootprint)
    _odom_msg.header.seq              ++;
    _odom_msg.header.stamp            = ros::Time::now();
    _odom_msg.header.frame_id         = "world"; // FIXME
    _odom_msg.child_frame_id          = "base_link"; // FIXME
    _odom_msg.pose.pose               = tf2::toMsg(_fb_pose);
    _odom_msg.twist.twist             = tf2::toMsg(_fbqdot);
    //tf2::eigenToCovariance(pose_cov,odom_msg_out_.pose.covariance);
    //tf2::eigenToCovariance(twist_cov,odom_msg_out_.twist.covariance);
    _odom_pub.publish(_odom_msg);
  }
  _t_prev = _t;

  rt_logger::RtLogger::getLogger().publish(ros::Time::now());
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
  _postural_error.fill(0.0);
  _postural_reference.fill(0.0);
  _postural_reference_dot.fill(0.0);

  _robot.reset(new wolf_controller::QuadrupedRobot(root_nh));

  rt_logger::RtLogger::getLogger().addPublisher("q_meas",_q_meas);
  rt_logger::RtLogger::getLogger().addPublisher("qdot_meas",_qdot_meas);
  rt_logger::RtLogger::getLogger().addPublisher("q",_q);
  rt_logger::RtLogger::getLogger().addPublisher("qdot",_qdot);
  rt_logger::RtLogger::getLogger().addPublisher("fbq",_fbq);
  rt_logger::RtLogger::getLogger().addPublisher("fbqdot",_fbqdot);
  rt_logger::RtLogger::getLogger().addPublisher("postural_error",_postural_error);
  rt_logger::RtLogger::getLogger().addPublisher("postural_reference",_postural_reference);
  rt_logger::RtLogger::getLogger().addPublisher("postural_reference_dot",_postural_reference_dot);

  message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
  message_filters::Subscriber<wolf_msgs::ContactForces> cf_sub;

  _odom_pub = root_nh.advertise<nav_msgs::Odometry>("/wolf_estimation/odom",100); // FIXME

  joint_state_sub.subscribe(root_nh,"/joint_states",100);
  imu_sub.subscribe(root_nh,"/trunk_imu/data",100);
  cf_sub.subscribe(root_nh,"/wolf_controller/contact_forces",100);
  _state_sub = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::Imu,wolf_msgs::ContactForces> >(joint_state_sub,imu_sub,cf_sub,100);
  _state_sub->registerCallback(update);

  //ros::Subscriber sub = root_nh.subscribe(_robot->getRobotName()+"/joint_states", 1000, update);

  ros::spin();

  return 0;
}
