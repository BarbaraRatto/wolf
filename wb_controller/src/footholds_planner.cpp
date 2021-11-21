#include <wb_controller/footholds_planner.h>

using namespace rt_logger;

namespace wb_controller {

FootholdsPlanner::FootholdsPlanner(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model, double step_length_max, double step_height_max)
{

  assert(gait_generator);
  gait_generator_ = gait_generator;
  assert(robot_model);
  robot_model_ = robot_model;
  assert(step_length_max>=0.0);
  step_length_max_ = step_length_max;
  assert(step_height_max>=0.0);
  step_height_max_ = step_height_max;

  hf_X_initial_footholds_.resize(4);
  hf_X_initial_hips_.resize(4); // \f$X_hip(i)\f$ with i corresponding to the leg number
  for(unsigned int i=0; i<4; i++)
  {
    hf_X_initial_hips_[i].setZero();
    hf_X_initial_footholds_[i].setZero();
  }

  offsets_applied_ = false;

  world_T_terrain_ = Eigen::Affine3d::Identity();

  push_recovery_ = std::make_shared<PushRecovery>(this);
  push_detected_ = false;
  push_recovery_active_ = false;

  step_height_ = 0.0; // [m]
  step_length_ = 0.0; // [m]

  base_linear_velocity_cmd_ = 0.0; // [m/s]
  base_angular_velocity_cmd_ = 0.0; // [rad/s]

  reset();

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/desired_height",base_position_(2));
}

void FootholdsPlanner::reset()
{

  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++)
  {
    steps_length_[foot_names[i]] = step_length_;
    steps_heading_[foot_names[i]] = 0.0;
    steps_height_[foot_names[i]] = step_height_;

    robot_model_->getPose(foot_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_); // base_T_foot
    tmp_matrix3d_ = robot_model_->getBaseRotationInHf(); // hf_R_base
    desired_foothold_[foot_names[i]]    = tmp_affine3d_.translation();
    virtual_foothold_[foot_names[i]]    = tmp_affine3d_.translation();
    current_foothold_hf_[foot_names[i]] = tmp_matrix3d_ * tmp_affine3d_.translation();
    current_foothold_[foot_names[i]]    = tmp_affine3d_.translation();
  }

  resetVelocyScales();

  robot_model_->getFloatingBasePose(tmp_affine3d_);
  base_rotation_reference_ = tmp_affine3d_.linear().transpose();
  rotTorpy(base_rotation_reference_,base_orientation_);
  base_position_ = tmp_affine3d_.translation();

  cmd_ = cmd_t::HOLD;

}

void FootholdsPlanner::update(const double& period, const Eigen::Vector3d& base_position) // OpenLoop Orientation
{
  update(period,base_position,base_orientation_);
}

void FootholdsPlanner::update(const double& period) // OpenLoop
{
  update(period,base_position_,base_orientation_);
}

void FootholdsPlanner::initializeFootPosition(const std::string& foot_name)
{
  robot_model_->getPose(foot_name,tmp_affine3d_); // world_T_foot
  gait_generator_->setInitialPose(foot_name,tmp_affine3d_);
}

void FootholdsPlanner::initializeFeetPosition()
{
  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();

  for(unsigned int i=0; i<foot_names.size(); i++)
    initializeFootPosition(foot_names[i]);
}

void FootholdsPlanner::update(const double& period, const Eigen::Vector3d& base_position, const Eigen::Vector3d& base_orientation) // ClosedLoop
{
  unsigned int cmd = cmd_;

  ROS_DEBUG_NAMED(CLASS_NAME,"update");

  world_R_hf_ = robot_model_->getHfRotationInWorld();
  hf_R_base_  = robot_model_->getBaseRotationInHf();

  setInitialOffsets();

  switch(cmd)
  {

  case cmd_t::HOLD:
    resetBaseVelocities();
    resetVelocyScales();
    calculateBasePosition(period,base_position);
    calculateBaseOrientation(period,base_orientation);
    break;

  case cmd_t::LINEAR:
    calculateBasePosition(period,base_position);
    resetBaseAngularVelocity();
    break;

  case cmd_t::ANGULAR:
    calculateBaseOrientation(period,base_orientation);
    resetBaseLinearVelocity();
    break;

  case cmd_t::LINEAR_AND_ANGULAR:
    calculateBasePosition(period,base_position);
    calculateBaseOrientation(period,base_orientation);
    break;

  case cmd_t::BASE_ONLY:
    calculateBasePosition(period,base_position);
    calculateBaseOrientation(period,base_orientation);
    break;

  case cmd_t::RESET_BASE:
    resetBaseVelocities();
    resetBasePosition();
    resetBaseOrientation();
    break;
  };

  if(push_recovery_active_ && push_recovery_->update(period))
  {
    push_detected_ = true;
    ROS_DEBUG_NAMED(CLASS_NAME,"Push detected!");
  }
  else
    push_detected_ = false;

  calculateFootSteps();

  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    // Set the initial pose for the next swing
    if(gait_generator_->isLiftOff(foot_names[i]))
      initializeFootPosition(foot_names[i]);

    gait_generator_->setStepLength(foot_names[i], steps_length_[foot_names[i]]);
    gait_generator_->setStepHeading(foot_names[i], steps_heading_[foot_names[i]]);
    gait_generator_->setStepHeight(foot_names[i], steps_height_[foot_names[i]]);
    gait_generator_->setStepHeadingRate(foot_names[i], steps_heading_rate_[foot_names[i]]);
  }
  gait_generator_->setTerrainRotation(world_T_terrain_.linear());

    if(robot_model_->getState() == QuadrupedRobot::robot_states_t::WALKING &&
       (cmd == cmd_t::LINEAR_AND_ANGULAR || push_detected_))
    {
      gait_generator_->activateSwing();
      //gait_generator_->setGaitType(Gait::gait_t::TROT);
    }
    else
    {
      gait_generator_->deactivateSwing();
    }

  // Update the gait_generator
  gait_generator_->update(period);
}

void FootholdsPlanner::calculateFootSteps()
{
  const std::vector<std::string>& foot_names = gait_generator_->getFootNames();

  for(unsigned int i=0; i<foot_names.size(); i++)
  {
    //if(gait_generator_->isLiftOff(feet_names[i]))
    if(gait_generator_->isSwinging(foot_names[i]))
    {
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"*********");
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"CalculateFootSteps for foot "<<foot_names[i]);

      // 1) Compute the displacement of the foot produced by the linear velocity command
      hf_delta_hip_.setZero(); // \f$\deltaL_{x,y,0}\f$
      hf_delta_hip_(0) = hf_base_linear_velocity_(0)*1.0/gait_generator_->getSwingFrequency(foot_names[i]);
      hf_delta_hip_(1) = hf_base_linear_velocity_(1)*1.0/gait_generator_->getSwingFrequency(foot_names[i]);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_ (Linear part): "<<hf_delta_hip_.transpose());

      // 2) Compute the displacement of the foot produced by the angular velocity command
      hf_delta_heding_.setZero(); // \f$\deltaL_{h,0}\f$
      hf_delta_heding_(2) = hf_base_angular_velocity_(2)*1.0/gait_generator_->getSwingFrequency(foot_names[i]);
      hf_delta_heding_ = hf_delta_heding_.cross(hf_X_initial_hips_[i]);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_heding_ (Angular part): "<<hf_delta_heding_.transpose());

      // 3) Combine the two displacements
      hf_delta_hip_(0)+= hf_delta_heding_(0);
      hf_delta_hip_(1)+= hf_delta_heding_(1);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_ (Combined): "<<hf_delta_hip_.transpose());

      // 4) Calculate the foothold offset based on the initial feet position (virtual foothold offset)
      robot_model_->getPose(foot_names[i],robot_model_->getBaseLinkName(),base_T_foot_);
      // current foot position in the horizontal frame
      hf_X_current_foothold_ = hf_R_base_ * base_T_foot_.translation();
      //world_X_virtual_foothold_offset_ = world_R_hf_ * (hf_X_initial_footholds_[i] - hf_X_current_foothold_);
      //world_X_virtual_foothold_offset_(2) = 0;
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_X_current_foothold_: "<<hf_X_current_foothold_.transpose());

      // 5) Sum everything to obtain the new foothold displacement w.r.t hf
      hf_delta_foot_.setZero();
      hf_delta_foot_.head(2) =  hf_delta_hip_.head(2)  + (hf_X_initial_footholds_[i] - hf_X_current_foothold_).head(2);

      //6) Sum delta com and the delta for the push recovery
      hf_delta_foot_.head(2) =  hf_delta_foot_.head(2) + push_recovery_->getDelta(foot_names[i]);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_foot_: "<<hf_delta_foot_.transpose());

      // 6) Sum everything to obtain the new foothold displacement w.r.t world
      //world_delta_foot_.setZero();
      //world_delta_foot_.head(2) =  world_R_hf_ * hf_delta_foot_;
      //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_delta_foot_: "<<world_delta_foot_.transpose());

      // 7) Get the step length and heading
      step_length_ = std::sqrt(hf_delta_foot_(0)*hf_delta_foot_(0) + hf_delta_foot_(1)*hf_delta_foot_(1));

      if(step_length_ > step_length_max_)
      {
        step_length_ = step_length_max_;
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Step length is greater than: "<<step_length_max_);
      }

      //desired_foothold_[foot_names[i]] = base_T_foot_ * hf_delta_foot_; // FIXME
      virtual_foothold_[foot_names[i]]    = hf_X_initial_footholds_[i];
      current_foothold_[foot_names[i]]    = base_T_foot_.translation();
      current_foothold_hf_[foot_names[i]] = hf_X_current_foothold_;

      steps_length_[foot_names[i]]         = step_length_;
      steps_heading_[foot_names[i]]        = std::atan2(hf_delta_foot_(1),hf_delta_foot_(0)) + robot_model_->getHfYawInWorld();
      steps_height_[foot_names[i]]         = step_height_;
      steps_heading_rate_[foot_names[i]]   = hf_base_angular_velocity_(2);

      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_length["<<foot_names[i]<<"]: "<<steps_length_[foot_names[i]]);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_["<<foot_names[i]<<"]: "<<steps_heading_[foot_names[i]]);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_height_["<<foot_names[i]<<"]: "<<steps_height_[foot_names[i]]);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_rate_["<<foot_names[i]<<"]: "<<steps_heading_rate_[foot_names[i]]);
    }
    else // if(gait_generator_->isTouchDown(feet_names[i]))
    {
      steps_length_[foot_names[i]]         = 0.0;
      steps_heading_[foot_names[i]]        = 0.0;
      steps_height_[foot_names[i]]         = 0.0;
      steps_heading_rate_[foot_names[i]]   = 0.0;
    }
  }
}

void FootholdsPlanner::resetFeetStep()
{
  const std::vector<std::string>& feet_names = gait_generator_->getFootNames();

  for(unsigned int i=0; i<feet_names.size(); i++)
  {
    steps_length_[feet_names[i]]   = 0.0;
    steps_heading_[feet_names[i]] = 0.0;
    steps_height_[feet_names[i]]   = 0.0;
    steps_heading_rate_[feet_names[i]]   = 0.0;
  }
}

void FootholdsPlanner::resetBaseAngularVelocity()
{
  hf_base_angular_velocity_.setZero();
  hf_base_angular_velocity_ref_.setZero();
  hf_base_angular_velocity_filt_.setZero();
}

void FootholdsPlanner::resetBaseLinearVelocity()
{
  hf_base_linear_velocity_.setZero();
  hf_base_linear_velocity_ref_.setZero();
  hf_base_linear_velocity_filt_.setZero();
}

void FootholdsPlanner::resetBaseVelocities()
{
  resetBaseAngularVelocity();
  resetBaseLinearVelocity();
}

void FootholdsPlanner::resetBasePosition()
{
  for(unsigned int i=0;i<3;i++)
    base_position_(i) = secondOrderFilter(base_position_(i),base_position_filt_(i),default_base_position_(i),0.5);
}

void FootholdsPlanner::resetBaseOrientation()
{
  default_base_orientation_(2) = base_orientation_(2); // Keep the same yaw

  for(unsigned int i=0;i<3;i++)
    base_orientation_(i) = secondOrderFilter(base_orientation_(i),base_orientation_filt_(i),default_base_orientation_(i),0.5); //FIXME hardcoded gain, it should be based on the sampling time

  rpyToRot(base_orientation_,base_rotation_reference_);
  base_rotation_reference_.transposeInPlace();
}

void FootholdsPlanner::resetVelocyScales()
{
  base_linear_velocity_scale_x_ = 0.0;
  base_linear_velocity_scale_y_ = 0.0;
  base_linear_velocity_scale_z_ = 0.0;

  base_angular_velocity_scale_roll_ = 0.0;
  base_angular_velocity_scale_pitch_ = 0.0;
  base_angular_velocity_scale_yaw_ = 0.0;
}

void FootholdsPlanner::calculateBasePosition(const double& period, const Eigen::Vector3d& base_position)
{
  base_position_ = base_position;

  hf_base_linear_velocity_ref_(0) = base_linear_velocity_cmd_ * base_linear_velocity_scale_x_;
  hf_base_linear_velocity_ref_(1) = base_linear_velocity_cmd_ * base_linear_velocity_scale_y_;
  hf_base_linear_velocity_ref_(2) = base_linear_velocity_cmd_ * base_linear_velocity_scale_z_;

  for(unsigned int i=0;i<3;i++)
    hf_base_linear_velocity_(i) = secondOrderFilter(hf_base_linear_velocity_(i),hf_base_linear_velocity_filt_(i),hf_base_linear_velocity_ref_(i),0.01); //FIXME hardcoded gain, it should be based on the sampling time

  base_linear_velocity_reference_ = world_R_hf_ * hf_base_linear_velocity_;

  base_position_ = base_linear_velocity_reference_ * period + base_position_;

  //base_position_reference_ = base_position_;

  // This is the base height reference computed w.r.t terrain ( reference = reference_world - reference_terrain )
  base_position_reference_.head(2) = base_position_.head(2);
  base_position_reference_(2) = base_position_(2) + world_T_terrain_.translation().z();
}

void FootholdsPlanner::calculateBaseOrientation(const double& period, const Eigen::Vector3d& base_orientation)
{
  base_orientation_ = base_orientation;

  hf_base_angular_velocity_ref_(0) = base_angular_velocity_cmd_ * base_angular_velocity_scale_roll_;
  hf_base_angular_velocity_ref_(1) = base_angular_velocity_cmd_ * base_angular_velocity_scale_pitch_;
  hf_base_angular_velocity_ref_(2) = base_angular_velocity_cmd_ * base_angular_velocity_scale_yaw_;

  for(unsigned int i=0;i<3;i++)
    hf_base_angular_velocity_(i) = secondOrderFilter(hf_base_angular_velocity_(i),hf_base_angular_velocity_filt_(i),hf_base_angular_velocity_ref_(i),0.5);

  base_angular_velocity_reference_ = hf_base_angular_velocity_;

  base_orientation_ = base_angular_velocity_reference_ * period + base_orientation_;

  rpyToRot(base_orientation_,base_rotation_reference_);
  // This is the base rotation reference computed w.r.t terrain
  base_rotation_reference_ = world_T_terrain_.linear().transpose() * base_rotation_reference_.transpose();
}

void FootholdsPlanner::setInitialOffsets()
{
  if(!offsets_applied_)
  {
    const std::vector<std::string>& hips_names = robot_model_->getHipNames();
    for(unsigned int i=0; i<hips_names.size(); i++)
    {
      robot_model_->getPose(gait_generator_->getFootNames()[i],robot_model_->getBaseLinkName(),tmp_affine3d_1_); // base_T_foot_
      robot_model_->getPose(hips_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_); // base_T_hip
      tmp_matrix3d_ = robot_model_->getBaseRotationInHf(); // hf_R_base_
      // initial feet offsets in the horizontal frame
      hf_X_initial_footholds_[i] = tmp_matrix3d_ * tmp_affine3d_1_.translation();
      // initial hip positions, we assume the base starts horizontal (TODO)
      hf_X_initial_hips_[i] = tmp_affine3d_.translation();
    }

    offsets_applied_ = true;
  }
}

void FootholdsPlanner::startPushRecovery(bool start)
{
  push_recovery_active_ = start;
  if(push_recovery_active_)
    push_recovery_->activateComputeDeltas();
  else
    push_recovery_->deactivateComputeDeltas();
}

void FootholdsPlanner::togglePushRecovery()
{
  push_recovery_active_ = !push_recovery_active_;
  if(push_recovery_active_)
    push_recovery_->activateComputeDeltas();
  else
    push_recovery_->deactivateComputeDeltas();
}

bool FootholdsPlanner::isPushRecoveryActive() const
{
  return push_recovery_active_;
}

// Sets
void FootholdsPlanner::setCmd(const unsigned int cmd)
{
  cmd_ = cmd;
}

void FootholdsPlanner::setBasePosition(const Eigen::Vector3d& position)
{
  base_position_ = position;
}

void FootholdsPlanner::setBaseOrientation(const Eigen::Vector3d& orientation)
{
  base_orientation_ = orientation;
}

void FootholdsPlanner::setDefaultBaseOrientation(const Eigen::Vector3d& orientation)
{
  default_base_orientation_ = orientation;
}

void FootholdsPlanner::setDefaultBasePosition(const Eigen::Vector3d& position)
{
  default_base_position_ = position;
}

void FootholdsPlanner::setBaseVelocityScaleX(const double& scale)
{
  base_linear_velocity_scale_x_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleY(const double& scale)
{
  base_linear_velocity_scale_y_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleZ(const double& scale)
{
  base_linear_velocity_scale_z_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleRoll(const double& scale)
{
  base_angular_velocity_scale_roll_ = scale;
}

void FootholdsPlanner::setBaseVelocityScalePitch(const double& scale)
{
  base_angular_velocity_scale_pitch_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleYaw(const double& scale)
{
  base_angular_velocity_scale_yaw_ = scale;
}

void FootholdsPlanner::increaseStepHeight()
{
  setStepHeight(step_height_ + 0.01); // Increase step height
}

void FootholdsPlanner::decreaseStepHeight()
{
  setStepHeight(step_height_ - 0.01); // Decrease step height
}

void FootholdsPlanner::setTerrainTransform(const Eigen::Affine3d &world_T_terrain)
{
  world_T_terrain_ = world_T_terrain;
}

void FootholdsPlanner::setPushRecoveryThresholds(const Eigen::Vector3d &static_th, const Eigen::Vector3d &dynamic_th)
{
  push_recovery_->setVelocityThresholds(static_th,dynamic_th);
}

void FootholdsPlanner::setPushRecoveryGains(const double &k_x, const double &k_y, const double &k_r)
{
  push_recovery_->setGains(k_x,k_y,k_r);
}

void FootholdsPlanner::setLinearVelocityCmd(const double& linear)
{
  base_linear_velocity_cmd_ = linear;
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set base linear velocity to "<< linear);
}

void FootholdsPlanner::setAngularVelocityCmd(const double& angular)
{
  base_angular_velocity_cmd_ = angular;
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set base angular velocity to "<< angular);
}

void FootholdsPlanner::setStepHeight(const double& height)
{
  if(height > step_height_max_) // Check if it is ok
  {
    double height_max = step_height_max_;
    step_height_ = height_max;
    ROS_WARN_STREAM_NAMED(CLASS_NAME,"Step height is greater than: "<<height_max);
  }
  else if(height <= 0.0)
  {
    step_height_ = 0.0;
    ROS_WARN_NAMED(CLASS_NAME,"Step height is less equal than: 0.0");
  }
  else
  {
    step_height_ = height;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set step height to: "<<height);
  }
}

void FootholdsPlanner::setMaxStepHeight(const double& max)
{
  if(max >= 0.0) // Check if it is ok
  {
    step_height_max_ = max;
  }
  else
    ROS_WARN_NAMED(CLASS_NAME,"Max step height is less equal than: 0.0");
}

void FootholdsPlanner::setMaxStepLength(const double& max)
{
  if(max >= 0.0) // Check if it is ok
  {
    step_length_max_ = max;
    push_recovery_->setMaxDelta(max);
  }
  else
    ROS_WARN_NAMED(CLASS_NAME,"Max step length is less equal than: 0.0");
}

// Gets
unsigned int FootholdsPlanner::getCmd()
{
  return cmd_;
}

const Eigen::Vector3d& FootholdsPlanner::getBasePositionReference() const
{
  return base_position_reference_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseLinearVelocityReference() const
{
  return base_linear_velocity_reference_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseAngularVelocityReference() const
{
  return base_angular_velocity_reference_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseLinearVelocityReferenceHF() const
{
  return hf_base_linear_velocity_ref_;
}

const Eigen::Vector3d& FootholdsPlanner::getBaseAngularVelocityReferenceHF() const
{
  return hf_base_angular_velocity_ref_;
}

const Eigen::Matrix3d& FootholdsPlanner::getBaseRotationReference() const
{
  return base_rotation_reference_;
}

const double& FootholdsPlanner::getStepLength(const std::string& foot_name)
{
  return steps_length_[foot_name];
}

const double& FootholdsPlanner::getStepHeading(const std::string& foot_name)
{
  return steps_heading_[foot_name];
}

const double& FootholdsPlanner::getStepHeight(const std::string& foot_name)
{
  return steps_height_[foot_name];
}

const double& FootholdsPlanner::getStepHeadingRate(const std::string& foot_name)
{
  return steps_heading_rate_[foot_name];
}

const double& FootholdsPlanner::getBaseHeight() const
{
  return base_position_reference_(2);
}

double FootholdsPlanner::getLinearVelocityCmd() const
{
  return base_linear_velocity_cmd_;
}

double FootholdsPlanner::getAngularVelocityCmd() const
{
  return base_angular_velocity_cmd_;
}

double FootholdsPlanner::getStepHeight() const
{
  return step_height_;
}

double FootholdsPlanner::getStepLength() const
{
  return step_length_;
}

Gait::gait_t FootholdsPlanner::getGaitType() const
{
  return gait_generator_->getGaitType();
}

bool FootholdsPlanner::isAnyFootInTouchDown()
{
  return gait_generator_->isAnyFootInTouchDown();
}

bool FootholdsPlanner::isAnyFootInSwing()
{
  return gait_generator_->isAnyFootInSwing();
}

bool FootholdsPlanner::areAllFeetInStance()
{
  return gait_generator_->areAllFeetInStance();
}

const std::vector<std::string>& FootholdsPlanner::getFootNames() const
{
  return gait_generator_->getFootNames();
}

double FootholdsPlanner::getSwingFrequency()
{
  return gait_generator_->getAvgSwingFrequency();
}

Eigen::Vector3d &FootholdsPlanner::getCurrentFoothold(const std::string &foot_name)
{
  return current_foothold_[foot_name];
}

Eigen::Vector3d &FootholdsPlanner::getCurrentFootholdHF(const std::string &foot_name)
{
  return current_foothold_hf_[foot_name];
}

Eigen::Vector3d &FootholdsPlanner::getVirtualFoothold(const std::string& foot_name)
{
  return virtual_foothold_[foot_name];
}

Eigen::Vector3d &FootholdsPlanner::getDesiredFoothold(const std::string& foot_name)
{
  return desired_foothold_[foot_name];
}

PushRecovery::PushRecovery(FootholdsPlanner* const footholds_planner_ptr)
{
  assert(footholds_planner_ptr);
  footholds_planner_ptr_ = footholds_planner_ptr;
  base_mass_   = footholds_planner_ptr_->robot_model_->getMass();
  base_length_ = footholds_planner_ptr_->robot_model_->getBaseLength();
  base_width_  = footholds_planner_ptr_->robot_model_->getBaseWidth();

  r_ = std::sqrt(base_length_*base_length_ + base_width_*base_length_)/2.0;
  rx_ = base_length_/2.0;
  ry_ = base_width_/2.0;

  // FIXME hardcoded foot names
  signs_["lf_foot"] = std::make_pair<int,int>(-1,1);
  signs_["rf_foot"] = std::make_pair<int,int>(1,1);
  signs_["lh_foot"] = std::make_pair<int,int>(-1,-1);
  signs_["rh_foot"] = std::make_pair<int,int>(1,-1);

  cutoff_freq_ = 20.0;
  th_filter_.setOmega(2.0*M_PI*cutoff_freq_);

  const std::vector<std::string>& foot_names = footholds_planner_ptr_->robot_model_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++)
  {
    deltas_[foot_names[i]].setZero();
    deltas_filter_[foot_names[i]].setOmega(2.0*M_PI*cutoff_freq_);
  }

  //velocity_filter_.setOmega(2.0*M_PI*cutoff_freq_);
  //velocity_filter_.setDamping(1.0);

  max_delta_ = 0.1 * footholds_planner_ptr_->step_length_max_;

  static_th_dot_(0) = 1000.0; // x [m/s]
  static_th_dot_(1) = 1000.0; // y [m/s]
  static_th_dot_(2) = 1000.0; // yaw [rad/s]

  dynamic_th_dot_(0) = 1000.0; // x [m/s]
  dynamic_th_dot_(1) = 1000.0; // y [m/s]
  dynamic_th_dot_(2) = 1000.0; // yaw [rad/s]

  current_th_dot_filt_ = current_th_dot_ = static_th_dot_;

  k_x_    = 0.0;
  k_y_    = 0.0;
  k_yaw_  = 0.0;

  compute_deltas_ = true;

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/current_th_dot_filt",current_th_dot_filt_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/cmd_velocity",cmd_velocity_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/base_velocity",base_velocity_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/base_velocity_filt",base_velocity_filt_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/error",error_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/delta_lf",deltas_["lf_foot"]);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/delta_rf",deltas_["rf_foot"]);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/delta_lh",deltas_["lh_foot"]);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/delta_rh",deltas_["rh_foot"]);
}

bool PushRecovery::update(const double& period)
{
  bool push_detected = true;

  //velocity_filter_.setTimeStep(period);
  th_filter_.setTimeStep(period);

  // Heuristic: scale the command velocity based on the gait type, that's because
  // the base velocity gets translated into foot step lengths so the real base velocity
  // is a reduced version of the command one.
  if(footholds_planner_ptr_->getGaitType() == Gait::TROT)
    cmd_velocity_scale_ = 0.5;
  else if (footholds_planner_ptr_->getGaitType() == Gait::CRAWL)
    cmd_velocity_scale_ = 0.25;
  else
    cmd_velocity_scale_ = 1.0;

  cmd_velocity_(0) = cmd_velocity_scale_*footholds_planner_ptr_->getBaseLinearVelocityReferenceHF()(0);
  cmd_velocity_(1) = cmd_velocity_scale_*footholds_planner_ptr_->getBaseLinearVelocityReferenceHF()(1);
  cmd_velocity_(2) = footholds_planner_ptr_->getBaseAngularVelocityReferenceHF()(2);

  footholds_planner_ptr_->robot_model_->getFloatingBaseTwist(base_twist_);
  base_twist_.head(3) = footholds_planner_ptr_->world_R_hf_.transpose() * base_twist_.head(3);

  footholds_planner_ptr_->robot_model_->getCOMVelocity(com_vel_hf_);
  com_vel_hf_ = footholds_planner_ptr_->world_R_hf_.transpose() * com_vel_hf_;

  // Note: we could use the com instead of the base because we control the com
  base_velocity_(0) = base_twist_(0);//com_vel_hf_(0);
  base_velocity_(1) = base_twist_(1);//com_vel_hf_(1);
  base_velocity_(2) = base_twist_(5);

  footholds_planner_ptr_->robot_model_->getFloatingBaseOrientationInertia(I_world_);
  I_hf_ = footholds_planner_ptr_->world_R_hf_.transpose() * I_world_;
  base_inertia_z_ = I_hf_(2,2);

  // Filter the base velocity
  //base_velocity_filt_ = velocity_filter_.process(base_velocity_);

  error_ = base_velocity_ - cmd_velocity_;

  if(cmd_velocity_.norm() > 0.0) // Check if the robot is moving
    current_th_dot_ = dynamic_th_dot_; // Apply the 'dynamic' threshold  i.e. higher bounds
  else
    current_th_dot_ = static_th_dot_; // Apply the 'static' threshold  i.e. lower bounds

  current_th_dot_filt_ = th_filter_.process(current_th_dot_);

  if(std::abs(error_(0)) < current_th_dot_filt_(0) && std::abs(error_(1)) < current_th_dot_filt_(1) && std::abs(error_(2)) < current_th_dot_filt_(2))
    push_detected = false;

  const std::vector<std::string>& foot_names = footholds_planner_ptr_->robot_model_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++) // Reset
    deltas_[foot_names[i]].setZero();

  if (compute_deltas_ && push_detected)
  {
    for(unsigned int i=0;i<foot_names.size();i++)
    {
      Z0h_  = std::abs(footholds_planner_ptr_->getCurrentFootholdHF(foot_names[i])(2));
      st_p_ = footholds_planner_ptr_->gait_generator_->getStancePeriod(foot_names[i]);
      tau_t_ = std::sqrt(Z0h_/GRAVITY);
      tau_r_ = std::sqrt(Z0h_*base_inertia_z_/base_mass_/GRAVITY/(r_*r_));

      Cx_pr_ =  tau_t_ * ( std::cosh(st_p_/tau_t_) / std::sinh(st_p_/tau_t_) - (1 - k_x_) / std::sinh(st_p_/tau_t_) );
      Cy_pr_ =  tau_t_ * ( std::cosh(st_p_/tau_t_) / std::sinh(st_p_/tau_t_) - (1 - k_y_) / std::sinh(st_p_/tau_t_) );
      Cr_pr_ =  tau_r_ * ( std::cosh(st_p_/tau_r_) / std::sinh(st_p_/tau_r_) - (1 - k_yaw_)  / std::sinh(st_p_/tau_r_) );

      deltas_[foot_names[i]].x() = Cx_pr_ * error_(0) + signs_[foot_names[i]].first  * Cr_pr_ * ry_ * error_(2);
      deltas_[foot_names[i]].y() = Cy_pr_ * error_(1) + signs_[foot_names[i]].second * Cr_pr_ * rx_ * error_(2);

      if(deltas_[foot_names[i]].x() > max_delta_)
        deltas_[foot_names[i]].x() = max_delta_;
      if(deltas_[foot_names[i]].x() < -max_delta_)
        deltas_[foot_names[i]].x() = -max_delta_;

      if(deltas_[foot_names[i]].y() > max_delta_)
        deltas_[foot_names[i]].y() = max_delta_;
      if(deltas_[foot_names[i]].y() < -max_delta_)
        deltas_[foot_names[i]].y() = -max_delta_;

      deltas_[foot_names[i]] = deltas_filter_[foot_names[i]].process(deltas_[foot_names[i]]);
    }
  }

  return push_detected;
}

const Eigen::Vector2d &PushRecovery::getDelta(const std::string &foot_name)
{
  return deltas_[foot_name];
}

void PushRecovery::setMaxDelta(const double& max)
{
  assert(max > 0);
  max_delta_ = max;
}

void PushRecovery::setVelocityThresholds(const Eigen::Vector3d &static_th, const Eigen::Vector3d &dynamic_th)
{
  static_th_dot_ = static_th;
  dynamic_th_dot_ = dynamic_th;
}

void PushRecovery::setGains(const double &k_x, const double &k_y, const double &k_yaw)
{
  assert(k_x>0.0 && k_x<=1.0);
  k_x_ = k_x;
  assert(k_y>0.0  && k_y<=1.0);
  k_y_ = k_y;
  assert(k_yaw>0.0 && k_yaw<=1.0);
  k_yaw_ = k_yaw;
}

void PushRecovery::activateComputeDeltas()
{
  compute_deltas_ = true;
  ROS_INFO_NAMED(CLASS_NAME,"Push recovery activated!");
}

void PushRecovery::deactivateComputeDeltas()
{
  compute_deltas_ = false;
  ROS_INFO_NAMED(CLASS_NAME,"Push recovery de-activated!");
}

}; // namespace
