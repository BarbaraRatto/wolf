#include <wb_controller/state_estimator.h>
#include <wb_controller/utils.h>

using namespace rt_logger;

namespace wb_controller {

StateEstimator::StateEstimator(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model)
{

  assert(robot_model);
  robot_model_ = robot_model;

  assert(gait_generator);
  gait_generator_ = gait_generator;

  const std::vector<std::string>& contact_names = robot_model_->getContactNames();
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& limb_names = robot_model_->getLimbNames();

  // Floating Base state estimation reset
  Eigen::Matrix6d contact_matrix;
  contact_matrix.setZero();
  contact_matrix.block(0,0,3,3) << Eigen::Matrix3d::Identity();
  qp_estimation_ = std::make_shared<OpenSoT::floating_base_estimation::qp_estimation>(robot_model_,foot_names,contact_matrix);

  int n_dofs = robot_model_->getJointNum();
  joint_positions_.resize(static_cast<Eigen::Index>(n_dofs));
  joint_velocities_.resize(static_cast<Eigen::Index>(n_dofs));
  joint_efforts_.resize(static_cast<Eigen::Index>(n_dofs));
  base_rpy_ = Eigen::Vector3d::Zero();
  floating_base_position_ = Eigen::Vector3d::Zero();
  floating_base_velocity_ = Eigen::Vector6d::Zero();
  floating_base_pose_ = Eigen::Affine3d::Identity();
  terrain_normal_ << 0,0,1;
  imu_orientation_.normalize();
  floating_base_velocity_qp_.resize(FLOATING_BASE_DOFS);
  contacts_estimation_active_ = false;

  for(unsigned int i=0;i<contact_names.size();i++)
  {
    contacts_[contact_names[i]] = true;
    contact_forces_[contact_names[i]] = Eigen::Vector3d::Zero();
    world_X_contact_[contact_names[i]] = Eigen::Vector3d::Zero();
  }

  mapRPYderivativesToOmega_ = Eigen::Matrix3d::Identity();
  base_R_world_ = Eigen::Matrix3d::Identity();
  raw_base_R_world_ = Eigen::Matrix3d::Identity();
  raw_base_rpy_ = Eigen::Vector3d::Zero();

  estimation_orientation_ = estimation_t::IMU_MAGNETOMETER;
  estimation_position_ = estimation_t::ESTIMATED_Z;

  estimations_["none"] = estimation_t::NONE;
  estimations_["imu_magnetometer"] = estimation_t::IMU_MAGNETOMETER;
  estimations_["imu_gyroscope"] = estimation_t::IMU_GYROSCOPE;
  estimations_["ground_truth"] = estimation_t::GROUND_TRUTH;
  estimations_["estimated_z"] = estimation_t::ESTIMATED_Z;

  reset_gyro_integration_done_ = false;

  haptic_contact_loop_active_ = false;

  contact_force_th_ = 0.0; // [N]

  use_external_forces_ = false;

  // Contact force estimation reset
  force_estimation_ = std::make_shared<XBot::Cartesian::Utils::ForceEstimationMomentumBased>(robot_model_,1.0/_period);
  //force_estimation_ = std::make_shared<XBot::Cartesian::Utils::ForceEstimation>(robot_model_->getXBotModel());

  // Contact estimation reset
  std::vector<int> dofs = {0,1,2}; // x y z
  std::vector<std::string> chain(1);

  assert(limb_names.size() == contact_names.size());
  for(unsigned int i=0;i<limb_names.size();i++)
  {
    chain[0] = limb_names[i];
    force_torque_sensors_[contact_names[i]] = force_estimation_->add_link(contact_names[i],dofs,chain);
  }

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/floating_base_position",floating_base_position_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/floating_base_velocity",floating_base_velocity_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/floating_base_velocity_qp",floating_base_velocity_qp_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/base_rpy",base_rpy_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/actual_height",floating_base_position_(2));
}

void StateEstimator::setEstimationType(const std::string& position_t, const std::string& orientation_t)
{
  setPositionEstimationType(position_t);
  setOrientationEstimationType(orientation_t);
}

void StateEstimator::setPositionEstimationType(const std::string& position_t)
{
  if(estimations_.find(position_t) == estimations_.end())
    ROS_WARN_STREAM_NAMED(CLASS_NAME,"Wrong position estimation type: "<<position_t);
  else
    setPositionEstimationType(estimations_[position_t]);
}

void StateEstimator::setOrientationEstimationType(const std::string& orientation_t)
{
  if(estimations_.find(orientation_t) == estimations_.end())
    ROS_WARN_STREAM_NAMED(CLASS_NAME,"Wrong orientation estimation type: "<<orientation_t);
  else
    setOrientationEstimationType(estimations_[orientation_t]);
}

void StateEstimator::setEstimationType(unsigned int position_t, unsigned int orientation_t)
{
  estimation_orientation_ = orientation_t;
  estimation_position_ = position_t;
}

void StateEstimator::setPositionEstimationType(unsigned int position_t)
{
  estimation_position_ = position_t;
}

void StateEstimator::setOrientationEstimationType(unsigned int orientation_t)
{
  estimation_orientation_ = orientation_t;
}

void StateEstimator::setJointPosition(const Eigen::VectorXd& joint_positions)
{
  joint_positions_ = joint_positions;
}

void StateEstimator::setJointVelocity(const Eigen::VectorXd& joint_velocities)
{
  joint_velocities_ = joint_velocities;
}

void StateEstimator::setJointEffort(const Eigen::VectorXd& joint_efforts)
{
  joint_efforts_ = joint_efforts;
}

void StateEstimator::setTerrainNormal(const Eigen::Vector3d &terrain_normal)
{
  terrain_normal_ = terrain_normal;
}

void StateEstimator::setImuOrientation(const Eigen::Quaterniond& imu_orientation)
{
  imu_orientation_ = imu_orientation;
}

void StateEstimator::setImuGyroscope(const Eigen::Vector3d& imu_gyroscope)
{
  imu_gyroscope_ = imu_gyroscope;
}

void StateEstimator::setGroundTruthBasePosition(const Eigen::Vector3d& gt_position)
{
  gt_position_ = gt_position;
}

void StateEstimator::setGroundTruthBaseOrientation(const Eigen::Quaterniond& gt_orientation)
{
  gt_orientation_ = gt_orientation;
}

void StateEstimator::setGroundTruthBaseLinearVelocity(const Eigen::Vector3d& gt_linear_velocity)
{
  gt_linear_velocity_ = gt_linear_velocity;
}

void StateEstimator::setGroundTruthBaseAngularVelocity(const Eigen::Vector3d& gt_angular_velocity)
{
  gt_angular_velocity_ = gt_angular_velocity;
}

void StateEstimator::setContactThreshold(const double& th)
{
  contact_force_th_ = th;
}

double StateEstimator::getContactThreshold()
{
  return contact_force_th_;
}

const std::string& StateEstimator::getPositionEstimationType()
{
  for (auto& tmp_map : estimations_)
    if(tmp_map.second == estimation_position_)
      return tmp_map.first;
}

const std::string& StateEstimator::getOrientationEstimationType()
{
  for (auto& tmp_map : estimations_)
    if(tmp_map.second == estimation_orientation_)
      return tmp_map.first;
}

void StateEstimator::setContactForces(const std::string& name, const Eigen::Vector3d& force)
{
  if(use_external_forces_ == false)
    use_external_forces_ = true;

  if(contact_forces_.count(name)> 0)
  {
    contact_forces_[name] = force;
  }
  else
  {
    ROS_WARN_NAMED(CLASS_NAME,"Wrong contact name!");
  }
}

const Eigen::Affine3d& StateEstimator::getFloatingBasePose() const
{
  return floating_base_pose_;
}

const Eigen::Vector3d& StateEstimator::getFloatingBasePosition() const
{
  return floating_base_position_;
}

const Eigen::Vector3d& StateEstimator::getFloatingBaseOrientationRPY() const
{
  return base_rpy_;
}

const std::map<std::string,Eigen::Vector3d>& StateEstimator::getContactForces() const
{
  return contact_forces_;
}

const std::map<std::string,bool>& StateEstimator::getContacts() const
{
  return contacts_;
}

const std::map<std::string,Eigen::Vector3d>& StateEstimator::getContactPositionInWorld() const
{
  return world_X_contact_;
}

void StateEstimator::toggleHapticContactLoop()
{
  haptic_contact_loop_active_=!haptic_contact_loop_active_;
}

void StateEstimator::startHapticContactLoop()
{
  haptic_contact_loop_active_ = true;
}

void StateEstimator::stopHapticContactLoop()
{
  haptic_contact_loop_active_ = false;
}

void StateEstimator::startContactsEstimation()
{
  contacts_estimation_active_ = true;

  ROS_INFO_NAMED(CLASS_NAME,"Start contact estimation");
}

void StateEstimator::resetGyroscopeIntegration()
{
  reset_gyro_integration_done_ = false;
}

void StateEstimator::stopContactsEstimation()
{
  contacts_estimation_active_ = false;

  ROS_INFO_NAMED(CLASS_NAME,"Stop contact estimation: the contacts state are forced to TRUE");
}

void StateEstimator::update(const double& period)
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& ee_names = robot_model_->getEndEffectorNames();

  for(unsigned int i=0; i<foot_names.size(); i++)
    world_X_contact_[foot_names[i]] = robot_model_->getFootPositionInWorld(foot_names[i]);

  for(unsigned int i=0; i<ee_names.size(); i++)
    world_X_contact_[ee_names[i]] = robot_model_->getEndEffectorPositionInWorld(ee_names[i]);

  updateContactState();

  updateFloatingBase(period);
}

void StateEstimator::updateContactState()
{
  force_estimation_->update();

  // Update contact state for the feet
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    if(!use_external_forces_)
    {
      tmp_vector3d_.setZero();
      force_torque_sensors_[foot_names[i]]->getForce(tmp_vector3d_); // tmp_vector3d_ = contact_force_foot
      contact_forces_[foot_names[i]] = robot_model_->getFootPoseInWorld(foot_names[i]) * tmp_vector3d_; // contact_force_world = world_T_foot * contact_force_foot
    }

    if(contacts_estimation_active_)
      contacts_[foot_names[i]] = (contact_forces_[foot_names[i]].dot(terrain_normal_) >= contact_force_th_ ? true : false);
    else
      contacts_[foot_names[i]] = true;

    if(haptic_contact_loop_active_)
    {
      qp_estimation_->setContactState(foot_names[i],contacts_[foot_names[i]]);
      gait_generator_->setContactState(foot_names[i],contacts_[foot_names[i]]);
    }
    else
    {
      qp_estimation_->setContactState(foot_names[i],gait_generator_->isTrajectoryFinished(foot_names[i]));
      gait_generator_->setContactState(foot_names[i],false);
    }
  }

  // Update contact state for the arm end-effectors
  const std::vector<std::string>& ee_names = robot_model_->getEndEffectorNames();
  for(unsigned int i=0; i<ee_names.size(); i++)
  {

    tmp_vector3d_.setZero();
    force_torque_sensors_[ee_names[i]]->getForce(tmp_vector3d_); // tmp_vector3d_ = contact_force_arm

    tmp_vector3d_ = robot_model_->getEndEffectorPoseInWorld(ee_names[i]) * tmp_vector3d_; // contact_force_world = world_T_ee * contact_force_ee

    if(contacts_estimation_active_)
      contacts_[ee_names[i]] = (tmp_vector3d_.norm() >= contact_force_th_ ? true : false);
    else
      contacts_[ee_names[i]] = false;

    contact_forces_[ee_names[i]] = tmp_vector3d_;
  }

}

double StateEstimator::estimateZ()
{
  // Estimate z using the legs position
  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();
  double estimated_z = 0.0;
  int feet_in_stance = 0;
  tmp_affine3d_.setIdentity();
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    if(!gait_generator_->isSwinging(foot_names[i]))
    {
      feet_in_stance++;
      tmp_affine3d_.translation() = floating_base_pose_.linear() *  robot_model_->getFootPositionInBase(foot_names[i]);
      estimated_z +=  tmp_affine3d_.translation().z();
    }
  }
  estimated_z /= feet_in_stance;
  return estimated_z;
}

void StateEstimator::updateFloatingBase(const double& period)
{
  unsigned int estimation_orientation = estimation_orientation_;
  unsigned int estimation_position = estimation_position_;

  // Update the joints information of the virtual model
  robot_model_->setJointVelocity(joint_velocities_);
  robot_model_->setJointEffort(joint_efforts_);
  robot_model_->setJointPosition(joint_positions_);

  // Note: we assume that the IMU is orientated as the base/waist of the robot
  // if this is not the case, it is necessary to add a transfomation from the IMU frame to the
  // base/waist frame

  switch(estimation_orientation)
  {
  case estimation_t::NONE:
    // The base does not rotate
    floating_base_pose_.linear() = Eigen::Matrix3d::Identity();
    floating_base_velocity_.segment(3,3) << 0.0, 0.0, 0.0;
    break;
  case estimation_t::IMU_MAGNETOMETER: // Use directly the orientation information from the IMU
    //use this with the real robot only if the magnetometer is not drifting
    quatToRotMat(imu_orientation_.normalized(),base_R_world_);
    rotTorpy(base_R_world_,base_rpy_);
    floating_base_pose_.linear() = base_R_world_.transpose();
    //IMU is in base frame in the real robot while in gazebo is in the world frame
#ifdef ROBOT_REAL
    floating_base_velocity_.segment(3,3) = floating_base_pose_.linear() * imu_gyroscope_;
#else
    floating_base_velocity_.segment(3,3) = imu_gyroscope_;
#endif
    break;
  case estimation_t::IMU_GYROSCOPE: // Intergate the gyroscope, useful if the magnetometer measure has interferences
    if(!reset_gyro_integration_done_) // Initialize the integration with the measured orientation
    {
      // Initialization for the integration
      quatToRotMat(imu_orientation_.normalized(),base_R_world_);
      rotTorpy(base_R_world_,base_rpy_);
      reset_gyro_integration_done_ = true;
    }
#ifdef ROBOT_REAL
    //IMU is in base frame in the real robot
    rpyToEarBase(base_rpy_,mapRPYderivativesToOmega_);
#else
    //IMU in gazebo is in the world frame
    rpyToEarWorld(base_rpy_,mapRPYderivativesToOmega_);
#endif
    // Map the omegas in the base into rpy derivatives and integrate
    base_rpy_ += (mapRPYderivativesToOmega_.inverse() * imu_gyroscope_) * period;

#ifdef ROBOT_REAL
    // Overwrite measures if one of them is more noisy (e.g. only yaw is noisy)
    quatToRotMat(imu_orientation_.normalized(),raw_base_R_world_);
    rotTorpy(raw_base_R_world_,raw_base_rpy_);
    base_rpy_.head(2) = raw_base_rpy_.head(2);
#endif

    //set the affine transformation for angular position
    rpyToRot(base_rpy_, base_R_world_);
    floating_base_pose_.linear() = base_R_world_.transpose();

    //set the affine transformation for angular velocity
#ifdef ROBOT_REAL
    //IMU is in base frame in the real robot
    floating_base_velocity_.segment(3,3) = base_R_world_.transpose() * imu_gyroscope_;
#else
    //IMU in gazebo is in the world frame
    floating_base_velocity_.segment(3,3) = imu_gyroscope_;
#endif

    break;
  case estimation_t::GROUND_TRUTH:
    quatToRotMat(gt_orientation_.normalized(),base_R_world_);
    rotTorpy(base_R_world_,base_rpy_);
    floating_base_pose_.linear() = base_R_world_.transpose();
    floating_base_velocity_.segment(3,3) = gt_angular_velocity_;
    break;
  default:
    // The base does not rotate
    floating_base_pose_.linear() = Eigen::Matrix3d::Identity();
    floating_base_velocity_.segment(3,3) << 0.0, 0.0, 0.0;
    break;
  };

  //// Reset the imu one time so that the world and the base are aligned
  ////tmp_matrix3d_ = world_R_imu_init_.transpose() * tmp_matrix3d_; // imu_R_world = R_imu0' * R_imu
  ////quatToRpy(floating_base_orientation_.normalized(),floating_base_orientation_rpy_);//take rpy measures

  // Update the orientation part of the floating base
  // This is necessary if we are going to use the qp estimator for the floating base linear velocities
  robot_model_->setFloatingBaseOrientation(floating_base_pose_.linear());
  robot_model_->setFloatingBaseAngularVelocity(floating_base_velocity_.segment(3,3));
  robot_model_->update();

  switch(estimation_position)
  {
  case estimation_t::NONE:
    // The base does not move
    floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
    floating_base_position_ << 0.0,0.0,0.0;
    break;
  case estimation_t::ESTIMATED_Z:
    // Update the qp estimation based on the new virtual model state
    //termporarily commented to save time
    qp_estimation_->update();
    qp_estimation_->getFloatingBaseTwist(floating_base_velocity_qp_);
    //floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
    //floating_base_velocity_.segment(0,3) << 0.0,0.0,floating_base_velocity_qp_(2);
    floating_base_velocity_.segment(0,3) = floating_base_velocity_qp_.segment(0,3);
    floating_base_position_ << 0.0,0.0, -estimateZ(); // Remove x and y from the state estimation
    break;
  case estimation_t::GROUND_TRUTH:
    floating_base_velocity_.segment(0,3) << gt_linear_velocity_;
    floating_base_position_.head(2) << gt_position_.head(2);
     // Note: this is the z calculated wrt the base not the one wrt world!
    floating_base_position_(2) = -estimateZ();
    break;
  default:
    // The base does not move
    floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
    floating_base_position_ << 0.0,0.0,0.0;
    break;
  };

  // Finally update the floating base with the full pose and velocities
  floating_base_pose_.translation() = floating_base_position_;
  robot_model_->setFloatingBaseState(floating_base_pose_,floating_base_velocity_); // This should trigger the update of the model
}

}
