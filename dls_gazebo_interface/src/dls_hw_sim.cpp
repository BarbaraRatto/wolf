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

        std::vector<std::string> joint_names(transmissions.size());
        // Initialize values from the transmission interface i.e. by using actuated joints (no floating base).
        for (unsigned int j=0; j < transmissions.size(); j++) {
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

            joint_names[j] = transmissions[j].joints_[0].name_;
        }

        if(!DlsRobotHwInterface::initializeInterfaces(joint_names))
        {
            ROS_ERROR_NAMED("dls_hw_sim","Initialization of DlsRobotHwInterface failed.");
            return false;
        }

        sim_model_ = parent_model;

        for (unsigned int j=0; j < n_dof_; j++) {

            gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
            if (!joint) {
                ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
                                 << "\" which is not in the gazebo model.");
                return false;
            }

            // Set limits
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

        ss_ = model_nh.advertiseService("freeze_base", &DlsRobotHwSim::freezeBase, this); //FIXME it should be moved to a dedicated interface
        freeze_base_sim_ = false;

        // Register interfaces
        registerInterface(&joint_state_adv_interface_);
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_interface_);
        registerInterface(&imu_sensor_interface_);
        registerInterface(&ground_truth_interface_);

        registerInterfaces();

        return true;
    }

    bool DlsRobotHwSim::registerInterfaces()
    {
        if(isInitialized())
        {
            // Register interfaces
            registerInterface(&joint_state_adv_interface_);
            registerInterface(&joint_state_interface_);
            registerInterface(&joint_interface_);
            registerInterface(&imu_sensor_interface_);
            registerInterface(&ground_truth_interface_);
        }
        return true;
    }

    bool DlsRobotHwSim::freezeBase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        //Freeze_base control
        freeze_base_sim_ = !freeze_base_sim_;
        if(freeze_base_sim_)
        {
                ROS_INFO("Freeze Base on!");
                sim_model_->SetWorldPose(inital_pose);
        }else{
                ROS_INFO("Freeze Base off!");
        }
        sim_model_->SetGravityMode(!freeze_base_sim_);

        return true;
    }

    void DlsRobotHwSim::readSim(ros::Time time, ros::Duration period)
    {

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

        //gazebo::math::Vector3  gzAngularAcc = sim_model_->GetWorldAngularAccel(); //not working
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
    }


    void DlsRobotHwSim::writeSim(ros::Time time, ros::Duration period)
    {

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

        //PDFF
        std::vector<double> ufb(n_dof_,0.0);
        std::vector<double> upd(n_dof_,0.0);
        std::vector<double> u_des(n_dof_,0.0);
        for (unsigned int i=0; i < n_dof_; i++) {
            ufb[i] += (joint_position_command_[i] - joint_position_[i]) * joint_p_gain_command_[i];
            ufb[i] += (joint_velocity_command_[i] - joint_velocity_[i]) * joint_d_gain_command_[i];
            u_des[i] = ufb[i] + joint_effort_command_[i];
        }

        for (unsigned int j=0; j < n_dof_; j++) {
            sim_joints_[j]->SetForce(0, u_des[j]);
        }

    }
}
