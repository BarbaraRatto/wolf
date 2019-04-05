#include <dls_hardware_interface/dls_robot_hw.h>

using namespace hardware_interface;

bool DlsRobotHwInterface::init(std::vector<transmission_interface::TransmissionInfo> transmissions)
{
    // We want to use only the motor/joints, use the transmission interface to get them.

    if(transmissions.size()<=0)
        return false;

    // Resize vectors to our DOF
    n_dof_ = transmissions.size();
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

    // Initialize values
    for (unsigned int j=0; j < n_dof_; j++) {
        // Check that this transmission has one joint
        if (transmissions[j].joints_.size() == 0) {
                ROS_WARN_STREAM_NAMED("dls_hw_sim","Transmission " << transmissions[j].name_
                                << " has no associated joints.");
                continue;
        } else if (transmissions[j].joints_.size() > 1) {
                ROS_WARN_STREAM_NAMED("dls_robot_hw_sim","Transmission " << transmissions[j].name_
                                << " has more than one joint. Currently the default robot hardware simulation "
                                << " interface only supports one.");
                continue;
        }

        // Check that this transmission has one actuator
        if (transmissions[j].actuators_.size() == 0) {
                ROS_WARN_STREAM_NAMED("dls_robot_hw_sim","Transmission " << transmissions[j].name_
                                << " has no associated actuators.");
                continue;
        } else if (transmissions[j].actuators_.size() > 1) {
                ROS_WARN_STREAM_NAMED("dls_robot_hw_sim","Transmission " << transmissions[j].name_
                                << " has more than one actuator. Currently the default robot hardware simulation "
                                << " interface only supports one.");
                continue;
        }

        // Add data from transmission
        joint_names_[j] = transmissions[j].joints_[0].name_;
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

    return true;
}
