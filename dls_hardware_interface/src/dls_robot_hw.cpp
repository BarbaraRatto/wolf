#include <dls_hardware_interface/dls_robot_hw.h>

using namespace hardware_interface;

DlsRobotHwInterface::DlsRobotHwInterface()
{
    initialized_ = false;
}

DlsRobotHwInterface::~DlsRobotHwInterface()
{
}

bool DlsRobotHwInterface::initializeInterfaces(const std::vector<std::string>& joint_names)
{
    // We want to use only the motor/joints.
    if(joint_names.size()<=0)
        return false;

    // Resize vectors to our DOF
    n_dof_ = static_cast<unsigned int>(joint_names.size());
    joint_names_.resize(n_dof_);
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

     for (unsigned int j=0; j < n_dof_; j++) {

        ROS_DEBUG_STREAM_NAMED("dls_robot_hw","Loading joint: "<< joint_names[j]);

        joint_names_[j] = joint_names[j];
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
                                                      joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j],
                                                      &joint_p_gain_[j], &joint_i_gain_[j], &joint_d_gain_[j]));

        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                                                  joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));


        joint_interface_.registerHandle(hardware_interface::JointCommandAdvHandle(
                                            joint_state_adv_interface_.getHandle(joint_names_[j]), &joint_position_command_[j],
                                            &joint_velocity_command_[j], &joint_effort_command_[j],
                                            &joint_p_gain_command_[j], &joint_i_gain_command_[j], &joint_d_gain_command_[j]));
    }

    // Devices hw interfaces
    imu_orientation_.resize(4);
    imu_ang_vel_.resize(3);
    imu_lin_acc_.resize(3);

    base_orientation_.resize(4);
    base_ang_vel_.resize(3);
    base_ang_vel_old_.resize(3);
    base_ang_acc_.resize(3);
    base_lin_acc_.resize(3);
    base_lin_pos_.resize(3);
    base_lin_vel_.resize(3);
    base_lin_vel_old_.resize(3);

    imuData.name = "trunk_imu"; // TODO: Fetch from elsewhere?
    imuData.frame_id = "imu"; // TODO: Fetch from URDF?
    imuData.orientation = &imu_orientation_[0];
    imuData.angular_velocity = &imu_ang_vel_[0];
    imuData.linear_acceleration = &imu_lin_acc_[0];
    imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imuData));

    gtData.name = "ground_truth"; // TODO: Fetch from elsewhere?
    gtData.frame_id = "base_link"; // TODO: Fetch from URDF?
    gtData.orientation = &base_orientation_[0];
    gtData.angular_velocity = &base_ang_vel_[0];
    gtData.angular_acceleration = &base_ang_acc_[0];
    gtData.linear_acceleration = &base_lin_acc_[0];
    gtData.linear_position = &base_lin_pos_[0];
    gtData.linear_velocity = &base_lin_vel_[0];
    ground_truth_interface_.registerHandle(hardware_interface::GroundTruthHandle(gtData));

    contact_sensor_names_.resize(4); // TODO: Fetch from elsewhere?
    contact_sensor_names_[0] = "lf_foot_contact_sensor";
    contact_sensor_names_[1] = "rf_foot_contact_sensor";
    contact_sensor_names_[2] = "lh_foot_contact_sensor";
    contact_sensor_names_[3] = "rh_foot_contact_sensor";
    /*contact_sensor_names_[4] = "lf_shin_contact_sensor";
    contact_sensor_names_[5] = "rf_shin_contact_sensor";
    contact_sensor_names_[6] = "lh_shin_contact_sensor";
    contact_sensor_names_[7] = "rh_shin_contact_sensor";*/
    // Create the handle for each contact sensor,
    contact_.resize(contact_sensor_names_.size());
    force_.resize(contact_sensor_names_.size());
    torque_.resize(contact_sensor_names_.size());
    normal_.resize(contact_sensor_names_.size());
    for (unsigned int i = 0; i < contact_sensor_names_.size(); i++)
    {
        contact_[i] = false;
        force_[i].resize(3,0);
        torque_[i].resize(3,0);
        normal_[i].resize(3,0);
        contact_sensor_interface_.registerHandle(hardware_interface::ContactSwitchSensorHandle(contact_sensor_names_[i], &contact_[i], &force_[i][0], &torque_[i][0], &normal_[i][0]));
    }

    initialized_ = true;

    return true;
}
