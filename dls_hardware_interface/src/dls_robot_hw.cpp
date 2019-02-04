#include <dls_hardware_interface/dls_robot_hw.h>

using namespace hardware_interface;

bool DlsRobotHwInterface::init()
{
    ros::NodeHandle nh; //Nodehandle without prefix
    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description))
    {
        ROS_ERROR("Could not find robot_description.");
        return false;
    }
    urdf::Model robot_model;
    if (!robot_model.initString(robot_description))
        ROS_ERROR("Failed to parse urdf file");

    // Resize vectors to our DOF
    n_dof_ = robot_model.joints_.size();
    joint_name_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_lower_limits_.resize(n_dof_);
    joint_upper_limits_.resize(n_dof_);
    joint_effort_limits_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_p_gain_.resize(n_dof_);
    joint_i_gain_.resize(n_dof_);
    joint_d_gain_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);
    joint_p_gain_command_.resize(n_dof_);
    joint_i_gain_command_.resize(n_dof_);
    joint_d_gain_command_.resize(n_dof_);

    // Add data from transmission
    joint_name_[j] = "Dummy"; //TODO
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_p_gain_[j] = 0;
    joint_i_gain_[j] = 0;
    joint_d_gain_[j] = 0;
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;
    joint_p_gain_command_[j] = 0.0;
    joint_i_gain_command_[j] = 0.0;
    joint_d_gain_command_[j] = 0.0;


    // Create joint state interface for all joints
    joint_state_adv_interface_.registerHandle(hardware_interface::JointStateAdvHandle(
                                                  joint_name_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j],
                                                  &joint_p_gain_[j], &joint_i_gain_[j], &joint_d_gain_[j]));

    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                                              joint_name_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));


    joint_interface_.registerHandle(hardware_interface::JointCommandAdvHandle(
                                        joint_state_adv_interface_.getHandle(joint_name_[j]), &joint_position_command_[j],
                                        &joint_velocity_command_[j], &joint_effort_command_[j],
                                        &joint_p_gain_command_[j], &joint_i_gain_command_[j], &joint_d_gain_command_[j]));


    // Register interfaces
    registerInterface(&joint_state_adv_interface_);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_interface_);
    /*registerInterface(&imu_sensor_interface_);
    registerInterface(&ground_truth_interface_);
    registerInterface(&force_sensor_interface_);
    registerInterface(&contact_sensor_interface_);
    registerInterface(&shin_sensor_interface_);
    registerInterface(&sim_interface_);
    registerInterface(&motor_interface_);*/

    return true;

}
