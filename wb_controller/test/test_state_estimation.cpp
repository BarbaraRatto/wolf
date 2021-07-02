// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "test_common_utils.h"
#include <OpenSoT/floating_base_estimation/qp_estimation.h>

OpenSoT::floating_base_estimation::qp_estimation::Ptr _qp_estimation;
wb_controller::QuadrupedRobot::Ptr _robot;
Eigen::VectorXd _joint_positions(18);
Eigen::VectorXd _joint_velocities(18);
Eigen::VectorXd _qdot(6);


void update(const sensor_msgs::JointState::ConstPtr& msg)
{

    for(unsigned int i =0;i<msg->name.size();i++)
    {
        _joint_positions(i) = msg->position[i];
        _joint_velocities(i) = msg->velocity[i];
    }

    _robot->getXBotModel()->setJointVelocity(_joint_velocities);
    _robot->getXBotModel()->setJointPosition(_joint_positions);
    _qp_estimation->update(OpenSoT::FloatingBaseEstimation::Update::All);

   _qp_estimation->getFloatingBaseTwist(_qdot);

   ROS_INFO_STREAM("Estimated FB Vel: "<< _qdot.transpose());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_state_estimation");
    ros::NodeHandle root_nh;

    _robot.reset(wb_controller::createRobotModel(root_nh));

    Eigen::Matrix6d contact_matrix; contact_matrix.setZero();
    contact_matrix.block(0,0,3,3) << Eigen::Matrix3d::Identity();
    _qp_estimation.reset(new OpenSoT::floating_base_estimation::qp_estimation(_robot->getXBotModel(),_robot->getFootNames(),contact_matrix));

    ros::Subscriber sub = root_nh.subscribe("/hyq/joint_states", 1000, update);

    ros::spin();

    return 0;
}
