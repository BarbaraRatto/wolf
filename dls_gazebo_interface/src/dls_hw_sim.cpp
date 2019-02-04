#include <dls_gazebo_interface/dls_hw_sim.h>

#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/math/Vector3.hh>

PLUGINLIB_EXPORT_CLASS(dls_gazebo_interface::DlsRobotHwSim, gazebo_ros_control::RobotHWSim)

namespace dls_gazebo_interface
{

    using namespace hardware_interface;

    bool DlsRobotHwSim::initSim(const std::string& robot_namespace,
                                ros::NodeHandle model_nh,
                                gazebo::physics::ModelPtr parent_model,
                                const urdf::Model *const urdf_model,
                                std::vector<transmission_interface::TransmissionInfo> transmissions)
    {
        const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

        sim_model_ = parent_model;

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

            const std::string &hardware_interface = transmissions[j].actuators_[0].hardware_interfaces_[0];

            // Debug
            ROS_DEBUG_STREAM_NAMED("dls_robot_hw_sim","Loading joint '" << joint_names_[j]
                                   << "' of type '" << hardware_interface << "'");

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

            gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
            if (!joint) {
                ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
                                 << "\" which is not in the gazebo model.");
                return false;
            }
            sim_joints_.push_back(joint);
            joint_effort_limits_[j] = joint->GetEffortLimit(0);
            // TODO this set is useless:
            // joint->SetEffortLimit(0,joint_effort_limits_[j]);

        }

        inital_pose = sim_model_->GetWorldPose();
        robot_name_ = sim_model_->GetName();

        // Hardware interfaces: Base IMU sensors
        imu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>
                (gazebo::sensors::SensorManager::Instance()->GetSensor("trunk_imu"));
        if (!this->imu_sensor_) 	{
            ROS_ERROR_STREAM("Could not find base IMU sensor.");
        }

        //foot switch
        foot_sensors_.resize(4);
        std::vector<std::string> foot_sensor_names(4);
        foot_sensor_names[0] = std::string("lf_foot_contact_sensor");
        foot_sensor_names[1] = std::string("rf_foot_contact_sensor");
        foot_sensor_names[2] = std::string("lh_foot_contact_sensor");
        foot_sensor_names[3] = std::string("rh_foot_contact_sensor");


        for (int n = 0; n < foot_sensors_.size(); n++) {
            foot_sensors_[n] = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>
                    (gazebo::sensors::SensorManager::Instance()->GetSensor(foot_sensor_names[n]));
            if (!this->foot_sensors_[n]) 	{
                ROS_ERROR_STREAM("Could not find foot sensor \"" << foot_sensor_names[n] << "\".");
            }
        }

        //shin contact sensor
        shin_sensor_.resize(4);
        std::vector<std::string> shin_sensor_names(4);
        shin_sensor_names[0] = std::string("lf_shin_contact_sensor");
        shin_sensor_names[1] = std::string("rf_shin_contact_sensor");
        shin_sensor_names[2] = std::string("lh_shin_contact_sensor");
        shin_sensor_names[3] = std::string("rh_shin_contact_sensor");

        for (int n = 0; n < shin_sensor_.size(); n++) {
            shin_sensor_[n] = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>
                    (gazebo::sensors::SensorManager::Instance()->GetSensor(shin_sensor_names[n]));
            if (!this->shin_sensor_[n]) 	{
                ROS_ERROR_STREAM("Could not find shin sensor \"" << shin_sensor_names[n] << "\".");
            }
        }
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

        leg_names_.resize(4);
        force_.resize(4);
        torque_.resize(4);
        contact_.resize(4);

        leg_names_[0] = std::string("lf");
        leg_names_[1] = std::string("rf");
        leg_names_[2] = std::string("lh");
        leg_names_[3] = std::string("rh");

        std::string frame("base_link"); //Some quick hack to pass the data over;

        for (int n = 0; n < leg_names_.size(); n++) {
            force_[n].resize(3,0);
            torque_[n].resize(3,0);
            force_sensor_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
                                                       leg_names_[n], frame, &force_[n][0], &torque_[n][0]));
            contact_sensor_interface_.registerHandle(hardware_interface::ContactSwitchSensorHandle(
                                                         leg_names_[n], &contact_[n]));
            shin_sensor_interface_.registerHandle(hardware_interface::ShinSensorHandle(
                                                      leg_names_[n], &shin_contact_position_[n]));
        }

        xenomai_switch_count_sim_ = -1;
        std::string sim_interface_name("sim"); //Some quick hack to pass the data over;
        sim_interface_.registerHandle(hardware_interface::SimulationHandle(sim_interface_name,
                                                                           &xenomai_switch_count_sim_,&is_robot_real_sim_,&pause_sim_,&reset_sim_,&freeze_base_sim_,
                                                                           &ext_force_sim_, &ext_torque_sim_));

        misc_sensors_.resize(16 * sim_joints_.size());
        misc_sensors_names_.resize(16 * sim_joints_.size());
        std::string motor_interface_name("motor");
        motor_interface_.registerHandle(hardware_interface::MotorHandle(motor_interface_name,
                                                                        &remove_motor_torque_offsets_, &misc_sensors_, &misc_sensors_names_));

        // Register interfaces
        registerInterface(&joint_state_adv_interface_);
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_interface_);
        registerInterface(&imu_sensor_interface_);
        registerInterface(&ground_truth_interface_);
        registerInterface(&force_sensor_interface_);
        registerInterface(&contact_sensor_interface_);
        registerInterface(&shin_sensor_interface_);
        registerInterface(&sim_interface_);
        registerInterface(&motor_interface_);

        return true;
    }


    void DlsRobotHwSim::simulationInterface(void)
    {
        //Pause control
        static bool pause_state = pause_sim_;
        if(pause_sim_ != pause_state)
        {
            pause_state = pause_sim_;
            std::cout << "Change state" << std::endl;
            sim_model_->GetWorld()->SetPaused(pause_sim_);
        }

        //Freeze_base control
        static bool freeze_state = freeze_base_sim_;
        if(freeze_base_sim_ != freeze_state)
        {
            freeze_state = freeze_base_sim_;
            if(freeze_state)
            {
                std::cout << "Freeze Base on!" << std::endl;
                sim_model_->SetWorldPose(inital_pose);
            }else{
                std::cout << "Freeze Base off!" << std::endl;
            }
            sim_model_->SetGravityMode(!freeze_state);
        }
        if(freeze_base_sim_)
        {
            sim_model_->SetWorldPose(inital_pose);
            gazebo::physics::LinkPtr base_link = sim_model_->GetLink("base_link");
            if(base_link != NULL){
                //Set velocities and accelerations only for the base link:
                base_link->SetLinearVel(gazebo::math::Vector3::Zero);
                base_link->SetLinearAccel(gazebo::math::Vector3::Zero);
                base_link->SetAngularVel(gazebo::math::Vector3::Zero);
                base_link->SetAngularAccel(gazebo::math::Vector3::Zero);
            }
        }

        //Reset control
        if(reset_sim_)
        {
            reset_sim_ = false;
            std::cout << "Reset World" << std::endl;
            sim_model_->GetWorld()->Reset();
        }

        //External disturbance
        static Eigen::Vector3d ext_force_state = ext_force_sim_;
        if(ext_force_sim_ != ext_force_state)
        {
            ext_force_state = ext_force_sim_;
            std::cout << "Apply force " << ext_force_sim_ << std::endl;

            gazebo::math::Vector3 temp(ext_force_sim_(0),ext_force_sim_(1),ext_force_sim_(2));
            //sim_model_->GetLink("trunk")->SetForce(temp);
        }

    }

    void DlsRobotHwSim::readSim(ros::Time time, ros::Duration period)
    {

        simulationInterface();

        if(remove_motor_torque_offsets_ == true) {
            std::cout << "Trying to remove offsets" << std::endl;
            remove_motor_torque_offsets_ = false;
        }

        for (unsigned int j=0; j < n_dof_; j++) {
            // Gazebo has an interesting API...
            if (joint_types_[j] == urdf::Joint::PRISMATIC) {
                joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
            } else {
                joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                                                                        sim_joints_[j]->GetAngle(0).Radian());
            }
            joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
            joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
        }


        //IMU data:
        gazebo::math::Quaternion imu_quat(1, 0, 0, 0);
        gazebo::math::Vector3 imu_ang_vel(0, 0, 0);
        gazebo::math::Vector3 imu_lin_acc(0, 0, 0);

        if(imu_sensor_ != NULL){
            imu_quat = imu_sensor_->Orientation();
            imu_ang_vel = imu_sensor_->AngularVelocity();
            imu_lin_acc = imu_sensor_->LinearAcceleration();
        }

        imu_orientation_[0] = imu_quat.w;
        imu_orientation_[1] = imu_quat.x;
        imu_orientation_[2] = imu_quat.y;
        imu_orientation_[3] = imu_quat.z;

        imu_ang_vel_[0] = imu_ang_vel.x;
        imu_ang_vel_[1] = imu_ang_vel.y;
        imu_ang_vel_[2] = imu_ang_vel.z;

        imu_lin_acc_[0] = imu_lin_acc.x;
        imu_lin_acc_[1] = imu_lin_acc.y;
        imu_lin_acc_[2] = imu_lin_acc.z;

        //Ground truth:

        gazebo::math::Vector3  gzLinearVel = sim_model_->GetWorldLinearVel();
        base_lin_vel_[0] = gzLinearVel.x;
        base_lin_vel_[1] = gzLinearVel.y;
        base_lin_vel_[2] = gzLinearVel.z;

        //gazebo::math::Vector3  gzLinearAcc = sim_model_->GetLink("base_link")->GetLinearAccel(); //not working
        base_lin_acc_[0] = (base_lin_vel_[0] - base_lin_vel_old_[0])/0.001;
        base_lin_acc_[1] = (base_lin_vel_[1] - base_lin_vel_old_[1])/0.001;
        base_lin_acc_[2] = (base_lin_vel_[2] - base_lin_vel_old_[2])/0.001;
        base_lin_vel_old_[0] = base_lin_vel_[0];
        base_lin_vel_old_[1] = base_lin_vel_[1];
        base_lin_vel_old_[2] = base_lin_vel_[2];

        gazebo::math::Vector3  gzAngularVel = sim_model_->GetWorldAngularVel();
        base_ang_vel_[0] = gzAngularVel.x;
        base_ang_vel_[1] = gzAngularVel.y;
        base_ang_vel_[2] = gzAngularVel.z;

        //gazebo::math::Vector3  gzAngularAcc = sim_model_->GetWorldAngularAccel();//not working
        base_ang_acc_[0] = (base_ang_vel_[0] - base_ang_vel_old_[0])/0.001;
        base_ang_acc_[1] = (base_ang_vel_[1] - base_ang_vel_old_[1])/0.001;
        base_ang_acc_[2] = (base_ang_vel_[2] - base_ang_vel_old_[2])/0.001;
        base_ang_vel_old_[0] = base_lin_vel_[0];
        base_ang_vel_old_[1] = base_lin_vel_[1];
        base_ang_vel_old_[2] = base_lin_vel_[2];

        gazebo::math::Pose gzPose = sim_model_->GetWorldPose();
        base_lin_pos_[0] = gzPose.pos.x;
        base_lin_pos_[1] = gzPose.pos.y;
        base_lin_pos_[2] = gzPose.pos.z;
        base_orientation_[0] = gzPose.rot.w;
        base_orientation_[1] = gzPose.rot.x;
        base_orientation_[2] = gzPose.rot.y;
        base_orientation_[3] = gzPose.rot.z;

        //get virtual foot switch
        for (int n = 0; n < foot_sensors_.size(); n++) {
            gazebo::msgs::Contacts contacts;
            contacts = foot_sensors_[n]->Contacts();
            //std::cout << "Sensor " << n << " Contacts " <<  contacts.contact_size() << std::endl;
            if (contacts.contact_size()>=1)
            {
                contact_[n] = true;
            } else {
                contact_[n] = false;
            }

        }

        std::vector<gazebo::physics::LinkPtr> local_link(4);
        local_link[0] = sim_model_->GetLink("lf_lowerleg");
        local_link[1] = sim_model_->GetLink("rf_lowerleg");
        local_link[2] = sim_model_->GetLink("lh_lowerleg");
        local_link[3] = sim_model_->GetLink("rh_lowerleg");

        //get virtual shin sensor
        for (int n = 0; n < shin_sensor_.size(); n++) {
            gazebo::msgs::Contacts shin_msg;
            shin_msg = shin_sensor_[n]->Contacts();
            if (shin_msg.contact_size()>=1)
            {
                gazebo::math::Pose link_pose = local_link[n]->GetWorldPose();
                gazebo::math::Vector3 position_lower_leg = link_pose.rot.RotateVectorReverse(
                            gazebo::math::Vector3(shin_msg.contact(0).position(0).x(),
                                                  shin_msg.contact(0).position(0).y(),
                                                  shin_msg.contact(0).position(0).z()) - link_pose.pos);

                //take the position along the lowerleg (x axis)
                shin_contact_position_[n] = position_lower_leg.x;
            } else {
                shin_contact_position_[n] = 0.0;
            }
        }
    }


    void DlsRobotHwSim::writeSim(ros::Time time, ros::Duration period)
    {
        //PDFF
        std::vector<double> ufb(n_dof_,0.0);
        std::vector<double> upd(n_dof_,0.0);
        std::vector<double> u_des(n_dof_,0.0);
        for (unsigned int i=0; i < n_dof_; i++) {
            ufb[i] += (joint_position_command_[i] - joint_position_[i]) * joint_p_gain_command_[i];
            ufb[i] += (joint_velocity_command_[i] - joint_velocity_[i]) * joint_d_gain_command_[i];
            u_des[i] = ufb[i] + joint_effort_command_[i];
        }
        // FROM SL
        //  case PDFF:
        //    for (i=1; i<=n_dofs; ++i) {
        //      ufb[i] = 0.0;
        // ufb[i] += (joint_des_state[i].th - joint_state[i].th) *
        //   controller_gain_th[i];
        // ufb[i] += (joint_des_state[i].thd - joint_state[i].thd) *
        //   controller_gain_thd[i];
        //      upd[i] = ufb[i];
        //      u[i]   = ufb[i] + joint_des_state[i].uff;
        //    }
        //    break;

        for (unsigned int j=0; j < n_dof_; j++) {
            sim_joints_[j]->SetForce(0, u_des[j]);
        }

    }


}
