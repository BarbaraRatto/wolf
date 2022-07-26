// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "test_common_utils.h"

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>

#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/floating_base/IMU.h>

wolf_controller::QuadrupedRobot::Ptr _robot;
Eigen::VectorXd _joint_positions(18);
Eigen::VectorXd _joint_velocities(18);
Eigen::VectorXd _qdot(6);
OpenSoT::AutoStack::Ptr _stack;
std::shared_ptr<OpenSoT::solvers::iHQP> _solver;
//OpenSoT::utils::InverseDynamics::Ptr _id;
std::list<OpenSoT::tasks::Aggregated::TaskPtr> _contact_tasks;
std::map<std::string, unsigned int> _map_tasks;
OpenSoT::tasks::Aggregated::Ptr _aggregated_tasks;

void update(const sensor_msgs::JointState::ConstPtr& msg)
{
   // for(unsigned int i =0;i<msg->name.size();i++)
   // {
   //     _joint_positions(i) = msg->position[i];
   //     _joint_velocities(i) = msg->velocity[i];
   // }
   //
   // _robot->setJointVelocity(_joint_velocities);
   // _robot->setJointPosition(_joint_positions);
   // _qp_estimation->update(OpenSoT::FloatingBaseEstimation::Update::All);
   //
   //_qp_estimation->getFloatingBaseTwist(_qdot);

   ROS_INFO_STREAM("Estimated FB Vel: "<< _qdot.transpose());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_state_estimation");
    ros::NodeHandle root_nh;

    _robot.reset(wolf_controller::createRobotModel(root_nh));

    Eigen::Matrix6d contact_matrix; contact_matrix.setZero();
    contact_matrix.block(0,0,3,3) << Eigen::Matrix3d::Identity();

    auto contact_links = _robot->getFootNames();

    //OpenSoT::utils::InverseDynamics::CONTACT_MODEL id_contact_type = OpenSoT::utils::InverseDynamics::CONTACT_MODEL::POINT_CONTACT;
    //_id = std::make_shared<OpenSoT::utils::InverseDynamics>(contact_links, *_robot, id_contact_type);

    for(unsigned int i = 0; i < contact_links.size(); ++i)
    {
        OpenSoT::tasks::floating_base::Contact::Ptr tmp =
            std::make_shared<OpenSoT::tasks::floating_base::Contact>
                (*_robot, contact_links[i], contact_matrix);
        _contact_tasks.push_back(tmp);
        _map_tasks[contact_links[i]] = i;
    }

    _aggregated_tasks = std::make_shared<OpenSoT::tasks::Aggregated>(_contact_tasks, 6);

    _stack = std::make_shared<OpenSoT::AutoStack>(_aggregated_tasks);

    _solver = std::make_shared<OpenSoT::solvers::iHQP>(_stack->getStack(),_stack->getBounds(),1e6);
    _solver->setSolverID("FloatingBaseEstimation");


    ros::Subscriber sub = root_nh.subscribe("/hyq/joint_states", 1000, update);

    ros::spin();

    return 0;
}
