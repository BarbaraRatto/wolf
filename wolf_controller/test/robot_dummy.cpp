#include "robot_dummy.h"

using namespace hardware_interface;
using namespace wolf_controller;

void RobotDummy::init()
{
  // Hardware interfaces: Joints
  auto joint_names = loadJointNamesFromSRDF();
  if(joint_names.size()>0)
  {
    WolfRobotHwInterface::initializeJointsInterface(joint_names);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_effort_interface_);
  }
  else
  {
    throw std::runtime_error("Failed to register joint interface.");
  }

  // Hardware interfaces: IMU
  auto imu_name = loadImuLinkNameFromSRDF();
  if(!imu_name.empty())
  {
    WolfRobotHwInterface::initializeImuInterface(imu_name);
    registerInterface(&imu_sensor_interface_);
  }
  else
  {
    throw std::runtime_error("Failed to register imu interface.");
  }
}

void RobotDummy::read()
{
  // ------
  // Joints
  // ------
  for (unsigned int jj = 0; jj < n_dof_; ++jj)
  {
      joint_position_[jj] = 0.0;
      joint_velocity_[jj] = 0.0;
      joint_effort_[jj]   = 0.0;
  }
  // ---
  // IMU
  // ---
  imu_orientation_[0] = 1.0;
  imu_orientation_[1] = 0.0;
  imu_orientation_[2] = 0.0;
  imu_orientation_[3] = 0.0;
  imu_ang_vel_[0]     = 0.0;
  imu_ang_vel_[1]     = 0.0;
  imu_ang_vel_[2]     = 0.0;
  imu_lin_acc_[0]     = 0.0;
  imu_lin_acc_[1]     = 0.0;
  imu_lin_acc_[2]     = 0.0;
}

void RobotDummy::write()
{
  for (unsigned int jj = 0; jj < n_dof_; ++jj)
    if(std::isnan(joint_effort_command_[jj]))
      throw std::runtime_error("NaN in joint_effort_command");
}
