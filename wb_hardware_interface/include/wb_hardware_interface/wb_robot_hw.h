#ifndef WB_ROBOT_HW_INTERFACE_H
#define WB_ROBOT_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <wb_hardware_interface/ground_truth_interface.h>
#include <wb_hardware_interface/contact_switch_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>
#include <deque>

namespace hardware_interface
{

class WbRobotHwInterface
{
public:

    const std::string CLASS_NAME = "WbRobotHwInterface";

    WbRobotHwInterface();
    virtual ~WbRobotHwInterface();

    bool initializeInterfaces(const std::vector<std::string>& joint_names);

    virtual bool registerInterfaces() = 0;

    std::string getRobotName() {return robot_name_;}
    unsigned int getNdof() {return n_dof_;}
    bool isInitialized() {return initialized_;}

protected:

    std::string robot_name_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::ImuSensorInterface imu_sensor_interface_;
    hardware_interface::GroundTruthInterface ground_truth_interface_;
    hardware_interface::ContactSwitchSensorInterface contact_sensor_interface_;
    hardware_interface::EffortJointInterface joint_effort_interface_;

    unsigned int n_dof_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> contact_sensor_names_;
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
    std::vector<std::vector<double> > normal_;
    std::deque<bool> contact_;


private:

    bool initialized_;
};

} //@namespace hardware_interface

#endif
