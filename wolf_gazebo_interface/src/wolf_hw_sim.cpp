#include <wolf_gazebo_interface/wolf_hw_sim.h>

#include <gazebo/sensors/SensorManager.hh>
#include <ignition/math/Vector3.hh>

PLUGINLIB_EXPORT_CLASS(wolf_gazebo_interface::WolfRobotHwSim, gazebo_ros_control::RobotHWSim)

namespace wolf_gazebo_interface
{

  using namespace hardware_interface;

  bool WolfRobotHwSim::initSim(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              gazebo::physics::ModelPtr parent_model,
                              const urdf::Model *const urdf_model,
                              std::vector<transmission_interface::TransmissionInfo> transmissions)
  {

    const auto& sensor_manager  = gazebo::sensors::SensorManager::Instance();

    std::vector<std::string> joint_names(transmissions.size());
    // Initialize values from the transmission interface i.e. by using actuated joints (no floating base).
    for (unsigned int j=0; j < transmissions.size(); j++)
    {
      // Check that this transmission has one joint
      if (transmissions[j].joints_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has no associated joints.");
        continue;
      }
      else if (transmissions[j].joints_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has more than one joint. Currently the default robot hardware simulation "
                              << " interface only supports one.");
        continue;
      }
      // Check that this transmission has one actuator
      if (transmissions[j].actuators_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has no associated actuators.");
        continue;
      }
      else if (transmissions[j].actuators_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has more than one actuator. Currently the default robot hardware simulation "
                              << " interface only supports one.");
        continue;
      }
      joint_names[j] = transmissions[j].joints_[0].name_;
    }

    if(!WolfRobotHwInterface::initializeInterfaces(joint_names))
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Initialization of WolfRobotHwInterface failed.");
      return false;
    }

    sim_model_ = parent_model;

    for (unsigned int j=0; j < n_dof_; j++) {

      gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
      if (!joint)
      {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"This robot has a joint named \"" << joint_names_[j]
                               << "\" which is not in the gazebo model.");
        return false;
      }

      // Set limits
      sim_joints_.push_back(joint);
      joint_effort_limits_[j] = joint->GetEffortLimit(0);
      // TODO this set is useless:
      // joint->SetEffortLimit(0,joint_effort_limits_[j]);

    }

    inital_pose = sim_model_->WorldPose();
    robot_name_ = sim_model_->GetName();

    // Hardware interfaces: Base IMU sensors
    imu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor_manager->GetSensor("trunk_imu"));
    if (!this->imu_sensor_)
      ROS_WARN_NAMED(CLASS_NAME,"Could not find base IMU sensor, using the ground truth to fill the IMU data instead.");

    // Hardware interfaces: Contact sensors
    for(unsigned int i=0; i < contact_sensor_names_.size(); i++)
    {
      contact_sensors_.push_back(std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor_manager->GetSensor(contact_sensor_names_[i])));
      if(!this->contact_sensors_.back())
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Could not find "<< contact_sensor_names_[i] <<" .");
    }

    // Freeze base service
    ss_ = model_nh.advertiseService("freeze_base", &WolfRobotHwSim::freezeBase, this); //FIXME it should be moved to a dedicated interface
    freeze_base_sim_ = false;

    registerInterfaces();

    return true;
  }

  bool WolfRobotHwSim::registerInterfaces()
  {
    if(isInitialized())
    {
      // Register interfaces
      registerInterface(&joint_state_interface_);
      registerInterface(&imu_sensor_interface_);
      registerInterface(&ground_truth_interface_);
      registerInterface(&contact_sensor_interface_);
      registerInterface(&joint_effort_interface_);
    }
    return true;
  }

  bool WolfRobotHwSim::freezeBase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    //Freeze_base control
    freeze_base_sim_ = !freeze_base_sim_;
    if(freeze_base_sim_)
    {
      ROS_INFO_NAMED(CLASS_NAME,"Freeze Base on!");
      sim_model_->SetWorldPose(inital_pose);
    }else{
      ROS_INFO_NAMED(CLASS_NAME,"Freeze Base off!");
    }
    sim_model_->SetGravityMode(!freeze_base_sim_);

    return true;
  }

  void WolfRobotHwSim::readSim(ros::Time /*time*/, ros::Duration period)
  {

    for (unsigned int j=0; j < n_dof_; j++) {
      // Gazebo has an interesting API...
      if (joint_types_[j] == urdf::Joint::PRISMATIC) {
        joint_position_[j] = sim_joints_[j]->Position(0);
      } else {
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                                                                sim_joints_[j]->Position(0));
      }
      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
    }

    //Ground truth:
    ignition::math::Vector3d  gzLinearVel = sim_model_->WorldLinearVel();
    base_lin_vel_[0] = gzLinearVel.X();
    base_lin_vel_[1] = gzLinearVel.Y();
    base_lin_vel_[2] = gzLinearVel.Z();

    //gazebo::math::Vector3  gzLinearAcc = sim_model_->GetLink("base_link")->GetLinearAccel(); //not working
    base_lin_acc_[0] = (base_lin_vel_[0] - base_lin_vel_old_[0])/period.toSec();
    base_lin_acc_[1] = (base_lin_vel_[1] - base_lin_vel_old_[1])/period.toSec();
    base_lin_acc_[2] = (base_lin_vel_[2] - base_lin_vel_old_[2])/period.toSec();
    base_lin_vel_old_[0] = base_lin_vel_[0];
    base_lin_vel_old_[1] = base_lin_vel_[1];
    base_lin_vel_old_[2] = base_lin_vel_[2];

    ignition::math::Vector3d  gzAngularVel = sim_model_->WorldAngularVel();
    base_ang_vel_[0] = gzAngularVel.X();
    base_ang_vel_[1] = gzAngularVel.Y();
    base_ang_vel_[2] = gzAngularVel.Z();

    //gazebo::math::Vector3  gzAngularAcc = sim_model_->GetWorldAngularAccel(); //not working
    base_ang_acc_[0] = (base_ang_vel_[0] - base_ang_vel_old_[0])/period.toSec();
    base_ang_acc_[1] = (base_ang_vel_[1] - base_ang_vel_old_[1])/period.toSec();
    base_ang_acc_[2] = (base_ang_vel_[2] - base_ang_vel_old_[2])/period.toSec();
    base_ang_vel_old_[0] = base_lin_vel_[0];
    base_ang_vel_old_[1] = base_lin_vel_[1];
    base_ang_vel_old_[2] = base_lin_vel_[2];

    ignition::math::Pose3d gzPose = sim_model_->WorldPose();
    base_lin_pos_[0] = gzPose.Pos().X();
    base_lin_pos_[1] = gzPose.Pos().Y();
    base_lin_pos_[2] = gzPose.Pos().Z();
    base_orientation_[0] = gzPose.Rot().W();
    base_orientation_[1] = gzPose.Rot().X();
    base_orientation_[2] = gzPose.Rot().Y();
    base_orientation_[3] = gzPose.Rot().Z();

    //IMU data:
    ignition::math::Quaterniond imu_quat(1, 0, 0, 0);
    ignition::math::Vector3d imu_ang_vel(0, 0, 0);
    ignition::math::Vector3d imu_lin_acc(0, 0, 0);

    // In this case we are using the IMU sensor which has angular velocities and accelerations defined wrt the trunk/base
    if(imu_sensor_ != nullptr)
    {
      imu_quat    = imu_sensor_->Orientation();
      imu_ang_vel = imu_sensor_->AngularVelocity();
      imu_lin_acc = imu_sensor_->LinearAcceleration();
    }
    // In this case we need to transform the angular velocities and accelerations from world measurements (Gazebo) to trunk/base (IMU)
    else
    {
      quaterniond_tmp_.w() = gzPose.Rot().W();
      quaterniond_tmp_.x() = gzPose.Rot().X();
      quaterniond_tmp_.y() = gzPose.Rot().Y();
      quaterniond_tmp_.z() = gzPose.Rot().Z();
      quaterniond_tmp_.normalize();
      world_R_base_ = quaterniond_tmp_.toRotationMatrix();

      imu_quat.W() = gzPose.Rot().W();
      imu_quat.X() = gzPose.Rot().X();
      imu_quat.Y() = gzPose.Rot().Y();
      imu_quat.Z() = gzPose.Rot().Z();

      vector3d_tmp_ << static_cast<double>(gzAngularVel.X()),
                       static_cast<double>(gzAngularVel.Y()),
                       static_cast<double>(gzAngularVel.Z());
      vector3d_tmp_ = world_R_base_.transpose() * vector3d_tmp_; // base_angular_vel = base_R_world * world_angular_vel

      imu_ang_vel.X() = vector3d_tmp_(0);
      imu_ang_vel.Y() = vector3d_tmp_(1);
      imu_ang_vel.Z() = vector3d_tmp_(2);

      vector3d_tmp_ << static_cast<double>(base_ang_acc_[0]),
                       static_cast<double>(base_ang_acc_[1]),
                       static_cast<double>(base_ang_acc_[2]);
      vector3d_tmp_ = world_R_base_.transpose() * vector3d_tmp_; // base_angular_acc = base_R_world * world_angular_acc

      imu_lin_acc.X() = vector3d_tmp_(0);
      imu_lin_acc.Y() = vector3d_tmp_(1);
      imu_lin_acc.Z() = vector3d_tmp_(2);
    }

    imu_orientation_[0] = imu_quat.W();
    imu_orientation_[1] = imu_quat.X();
    imu_orientation_[2] = imu_quat.Y();
    imu_orientation_[3] = imu_quat.Z();

    imu_ang_vel_[0] = imu_ang_vel.X();
    imu_ang_vel_[1] = imu_ang_vel.Y();
    imu_ang_vel_[2] = imu_ang_vel.Z();

    imu_lin_acc_[0] = imu_lin_acc.X();
    imu_lin_acc_[1] = imu_lin_acc.Y();
    imu_lin_acc_[2] = imu_lin_acc.Z();

    // FIXME We need the lowerleg links for the transfrom from the feet
    if(contact_sensors_.size() == 4) // We assume we are working only with the feet
    {

      std::vector<gazebo::physics::LinkPtr> lowerleg_link(4);
      lowerleg_link[0] = sim_model_->GetLink("lf_lowerleg");
      lowerleg_link[1] = sim_model_->GetLink("rf_lowerleg");
      lowerleg_link[2] = sim_model_->GetLink("lh_lowerleg");
      lowerleg_link[3] = sim_model_->GetLink("rh_lowerleg");

      // Fill the contact sensors reading
      for (unsigned int i = 0; i < contact_sensors_.size(); i++)
      {
        gazebo::msgs::Contacts contacts;
        contacts = contact_sensors_[i]->Contacts();

        if (contacts.contact_size()>=1)
        {
          contact_[i] = true;
          //FIXME the wrench is in the last link where the foot is lumped that is the lowerleg! so it is expressed in the lowerleg
          //map from lowerleg frame to world
          ignition::math::Pose3d link_pose = lowerleg_link[i]->WorldPose();
          ignition::math::Vector3d forceW = link_pose.Rot().RotateVector(
                ignition::math::Vector3d(contacts.contact(0).wrench(0).body_1_wrench().force().x(),
                                         contacts.contact(0).wrench(0).body_1_wrench().force().y(),
                                         contacts.contact(0).wrench(0).body_1_wrench().force().z()));

          // These forces are in the world frame!
          force_[i][0] = forceW.X();
          force_[i][1] = forceW.Y();
          force_[i][2] = forceW.Z();
          // The normal is expressed in the world frame!
          normal_[i][0]  = contacts.contact(0).normal(0).x();
          normal_[i][1]  = contacts.contact(0).normal(0).y();
          normal_[i][2]  = contacts.contact(0).normal(0).z();
        }
        else
        {
          contact_[i] = false;
          force_[i][0]=0.0;
          force_[i][1]=0.0;
          force_[i][2]=0.0;
          normal_[i][0]  = 0.0;
          normal_[i][1]  = 0.0;
          normal_[i][2]  = 0.0;
        }

      }
    }

  }


  void WolfRobotHwSim::writeSim(ros::Time time, ros::Duration period)
  {

    if(freeze_base_sim_)
    {
      sim_model_->SetWorldPose(inital_pose);
      gazebo::physics::LinkPtr base_link = sim_model_->GetLink("base_link");
      if(base_link != nullptr){
        //Set velocities and accelerations only for the base link:
        base_link->SetLinearVel(ignition::math::Vector3d::Zero);
        //base_link->SetLinearAccel(ignition::math::Vector3d::Zero); // Deprecated in gazebo9
        base_link->SetAngularVel(ignition::math::Vector3d::Zero);
        //base_link->SetAngularAccel(ignition::math::Vector3d::Zero); // Deprecated in gazebo9
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
