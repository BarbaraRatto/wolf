#ifndef DLS_ROBOT_HW_INTERFACE_H
#define DLS_ROBOT_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <dls_hardware_interface/ground_truth_interface.h>
#include <dls_hardware_interface/joint_command_adv_interface.h>
#include <dls_hardware_interface/joint_state_adv_interface.h>
#include <dls_hardware_interface/simulation_interface.h>
#include <dls_hardware_interface/motor_interface.h>
#include <dls_hardware_interface/shin_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <dls_hardware_interface/contact_switch_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

namespace hardware_interface
{

class DlsRobotHwInterface
{
  public:
    DlsRobotHwInterface(){}
    ~DlsRobotHwInterface(){}

    //virtual void freezeBase(int flag) = 0;

    std::string getRobotName() {return robot_name_;}

  protected:



    std::string robot_name_;

    //hardware_interface::JointStateInterface test_;
    hardware_interface::JointStateAdvInterface joint_state_adv_interface_;
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::JointCommandAdvInterface joint_interface_;
    hardware_interface::ImuSensorInterface imu_sensor_interface_;
    hardware_interface::GroundTruthInterface ground_truth_interface_;
    hardware_interface::ForceTorqueSensorInterface force_sensor_interface_;
    hardware_interface::ContactSwitchSensorInterface contact_sensor_interface_;
    hardware_interface::ShinSensorInterface shin_sensor_interface_;
    hardware_interface::SimulationInterface sim_interface_;
    hardware_interface::MotorInterface motor_interface_;
    //hardware_interface::PosVelJointInterface posvel_command_interface;

    //Simulation interface variables:
    int xenomai_switch_count_sim_;
    bool is_robot_real_sim_;
    bool freeze_base_sim_;
    bool pause_sim_;
    bool reset_sim_;
    Eigen::Vector3d ext_force_sim_;
    Eigen::Vector3d ext_torque_sim_;

    //Motor interface variables:
    bool remove_motor_torque_offsets_;
    std::vector<double> misc_sensors_;
    std::vector<std::string> misc_sensors_names_;
    std::vector<std::string> leg_names_;

    std::vector<std::string> joint_names_;
    std::vector<int> joint_types_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    std::vector<double> joint_effort_limits_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_p_gain_;
    std::vector<double> joint_i_gain_;
    std::vector<double> joint_d_gain_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_p_gain_command_;
    std::vector<double> joint_i_gain_command_;
    std::vector<double> joint_d_gain_command_;

    hardware_interface::GroundTruthHandle::Data gtData;
    std::vector<double> base_orientation_;
    std::vector<double>	base_ang_vel_;
    std::vector<double> base_ang_vel_old_;
    std::vector<double>	base_ang_acc_;
    std::vector<double>	base_lin_acc_;
    std::vector<double>	base_lin_pos_;
    std::vector<double>	base_lin_vel_;
    std::vector<double> base_lin_vel_old_;

    hardware_interface::ImuSensorHandle::Data imuData;
    std::vector<double> imu_orientation_;
    std::vector<double>	imu_ang_vel_;
    std::vector<double>	imu_lin_acc_;

    std::vector<std::string> leg_name_;
    std::vector<std::vector<double> > force_;
    std::vector<std::vector<double> > torque_;
    std::deque<bool> contact_;
    std::deque<double> shin_contact_position_;

};

} //@namespace hardware_interface

#endif
