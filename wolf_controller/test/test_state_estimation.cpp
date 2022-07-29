// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "test_common_utils.h"

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/utils/Affine.h>

#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/tasks/velocity/Postural.h>

Eigen::VectorXd _q(18);
Eigen::VectorXd _qdot(18);
Eigen::VectorXd _fbq(6);

wolf_controller::QuadrupedRobot::Ptr _robot;
OpenSoT::AutoStack::Ptr _stack;
std::shared_ptr<OpenSoT::solvers::iHQP> _solver;
std::list<OpenSoT::tasks::Aggregated::TaskPtr> _contact_tasks;
OpenSoT::tasks::velocity::Postural::Ptr _postural;
OpenSoT::tasks::velocity::Cartesian::Ptr _imu_task;
OpenSoT::tasks::Aggregated::Ptr _aggregated_tasks;
Eigen::VectorXd _x;

void update(const sensor_msgs::JointState::ConstPtr& joints_msg)
{
  for(unsigned int i =0;i<joints_msg->name.size();i++)
  {
    _q(i+FLOATING_BASE_DOFS) = joints_msg->position[i];
    _qdot(i+FLOATING_BASE_DOFS) = joints_msg->velocity[i];
  }

  _robot->setJointPosition(_q);
  _robot->setJointVelocity(_qdot);

  _postural->setReference(_q,_qdot);

  _solver->solve(_x);

  ROS_INFO_STREAM("Solution: "<< _x.segment(0,6).transpose());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_state_estimation");
  ros::NodeHandle root_nh;

  _robot.reset(wolf_controller::createRobotModel(root_nh));

  _q.fill(0.0);
  _qdot.fill(0.0);

  _robot->getJointPosition(_q);

  auto contact_links = _robot->getFootNames();
  for(unsigned int i = 0; i < contact_links.size(); ++i)
  {
    auto tmp = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(contact_links[i]+"_task",_q,*_robot,contact_links[i],WORLD_FRAME_NAME);
    _contact_tasks.push_back(tmp);
  }
  _aggregated_tasks = std::make_shared<OpenSoT::tasks::Aggregated>(_contact_tasks, _q.size());

  _imu_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>("imu_task",_q,*_robot,_robot->getImuSensorName(),WORLD_FRAME_NAME);

  _postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
  _postural->setLambda(0.001);

  std::list<unsigned int> id_legs;
  id_legs.resize(12);
  std::list<unsigned int>::iterator it;
  unsigned int idx = FLOATING_BASE_DOFS;
  for (it = id_legs.begin(); it != id_legs.end(); ++it)
  {
      *it = idx;
      idx++;
  }

  _stack = std::make_shared<OpenSoT::AutoStack>(_imu_task+_aggregated_tasks);
  _stack << _postural%id_legs;

  _solver = std::make_shared<OpenSoT::solvers::iHQP>(_stack->getStack(),_stack->getBounds(),1e6);
  _solver->setSolverID("FloatingBaseEstimation");

  ros::Subscriber sub = root_nh.subscribe(_robot->getRobotName()+"/joint_states", 1000, update);

  ros::spin();

  return 0;
}
