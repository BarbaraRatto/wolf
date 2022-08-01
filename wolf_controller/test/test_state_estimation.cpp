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

Eigen::VectorXd _q(18);
Eigen::VectorXd _qdot(18);
Eigen::VectorXd _fbq(6);

static wolf_controller::QuadrupedRobot::Ptr _robot;
static OpenSoT::AutoStack::Ptr _stack;
static std::unique_ptr<OpenSoT::solvers::iHQP> _solver;
static std::map<std::string,OpenSoT::tasks::velocity::Cartesian::Ptr> _contact_tasks;
static OpenSoT::tasks::velocity::Postural::Ptr _postural;
static OpenSoT::tasks::velocity::Cartesian::Ptr _imu_task;
static OpenSoT::tasks::Aggregated::Ptr _aggregated_contacts;
static Eigen::VectorXd _x;
static std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::Imu,wolf_controller::ContactForces>> _state_sub;

void update(const sensor_msgs::JointState::ConstPtr& joints_msg,
            const sensor_msgs::Imu::ConstPtr& imu_msg,
            const wolf_controller::ContactForces::ConstPtr& cf_msg)
{
  for(unsigned int i =0;i<joints_msg->name.size();i++)
  {
    _q(i+FLOATING_BASE_DOFS) = joints_msg->position[i];
    _qdot(i+FLOATING_BASE_DOFS) = joints_msg->velocity[i];
  }

  _robot->setJointPosition(_q);
  _robot->setJointVelocity(_qdot);
  _robot->update();

  _postural->setReference(_q,_qdot);

  Eigen::Quaterniond q;
  q.w() = imu_msg->orientation.w;
  q.x() = imu_msg->orientation.x;
  q.y() = imu_msg->orientation.y;
  q.z() = imu_msg->orientation.z;

  Eigen::Affine3d imu_pose;
  imu_pose.linear() = q.toRotationMatrix();

  _imu_task->setReference(imu_pose); // FIXME missing linear and angular velocities

  for(unsigned int i=0; i<cf_msg->name.size(); i++)
  {
    if(cf_msg->contact[i])
      ROS_INFO_STREAM("Contact active: "<< cf_msg->name[i]);

    _contact_tasks[cf_msg->name[i]]->setActive(cf_msg->contact[i]);
    _contact_tasks[cf_msg->name[i]]->update(Eigen::VectorXd(1));
  }

  if(_solver->solve(_x))
    ROS_INFO_STREAM("Solution: "<< _x.segment(0,6).transpose());
  else
    ROS_WARN("Can not solve!");

  // Update the FB
  //Eigen::Affine3d fb_pose;
  //Eigen::Matrix3d fb_R;
  //fb_pose.translation() << _x.segment(0,3);
  //wolf_controller::rpyToRot(_x.segment(3,3),fb_R);
  //fb_pose.linear() << fb_R;
  //_robot->setFloatingBaseState(fb_pose,Eigen::Vector6d::Zero());
  //_robot->update();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_state_estimation");
  ros::NodeHandle root_nh;

  _robot.reset(wolf_controller::createRobotModel(root_nh));

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

  _q.fill(0.0);
  _qdot.fill(0.0);

  _robot->getJointPosition(_q);

  auto contact_links = _robot->getFootNames();
  for(unsigned int i = 0; i < contact_links.size(); ++i)
  {
    _contact_tasks[contact_links[i]] = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(contact_links[i],_q,*_robot,contact_links[i],WORLD_FRAME_NAME);
    _contact_tasks[contact_links[i]]->setLambda(0.0);
  }

  _aggregated_contacts = std::make_shared<OpenSoT::tasks::Aggregated>(_contact_tasks[contact_links[0]]%id_XYZ,_q.size());
  for(unsigned int i=1;i<contact_links.size();i++)
    _aggregated_contacts = _aggregated_contacts + _contact_tasks[contact_links[i]]%id_XYZ;

  _imu_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(_robot->getImuSensorName(),_q,*_robot,_robot->getImuSensorName(),WORLD_FRAME_NAME);

  _postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
  _postural->setLambda(0.001);

  _stack /= _imu_task%id_RPY + _aggregated_contacts;
  _stack << _postural%id_joints;
  _stack->update(Eigen::VectorXd(1));

  _solver = std::make_unique<OpenSoT::solvers::iHQP>(_stack->getStack(),_stack->getBounds(),1e6);
  _solver->setSolverID("FloatingBaseEstimation");

  message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
  message_filters::Subscriber<wolf_controller::ContactForces> cf_sub;

  joint_state_sub.subscribe(root_nh,_robot->getRobotName()+"/joint_states",20);
  imu_sub.subscribe(root_nh,"/trunk_imu/data",20);
  cf_sub.subscribe(root_nh,_robot->getRobotName()+"/wolf_controller/contact_forces",20);
  _state_sub = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::Imu,wolf_controller::ContactForces> >(joint_state_sub,imu_sub,cf_sub,20);
  _state_sub->registerCallback(update);


  //ros::Subscriber sub = root_nh.subscribe(_robot->getRobotName()+"/joint_states", 1000, update);

  ros::spin();

  return 0;
}
