#include "wb_controller/legs_kinematics.h"

namespace wb_controller {

LegsKinematics::LegsKinematics(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model, TerrainEstimator::Ptr terrain_estimator)
  : clik_gain_(1.0)
{

  assert(gait_generator);
  gait_generator_ = gait_generator;
  assert(robot_model);
  robot_model_ = robot_model;
  assert(terrain_estimator);
  terrain_estimator_ = terrain_estimator;

  // Set home position, qmin and qmax defined in the srdf
  // Initial values
  robot_model_->getRobotState("home", qhome_);
  robot_model_->setJointPosition(qhome_);

  des_joint_positions_.resize(qhome_.size());
  des_joint_velocities_.resize(qhome_.size());

  I_ = Eigen::Matrix3d::Identity();
  damp_max_ = 0.001;
  determinant_max_ = 0.1;

  reset();
}

void LegsKinematics::setDesiredFootPositions(const std::string& foot_name, const Eigen::Vector3d& position)
{
    desired_foot_positions_[foot_name] = position;
}

bool LegsKinematics::update(const double& period, const Eigen::VectorXd& current_joint_positions)
{

  des_joint_velocities_.fill(0.0);

  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& leg_names = robot_model_->getLegNames();

  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    robot_model_->getJacobian(foot_names[i],J_); //wrt WORLD
    robot_model_->getPose(robot_model_->getBaseLinkName(),world_T_base_);

    int idx = robot_model_->getLimbJointsIds(leg_names[i])[0]; // NOTE: take the first idx because the leg joints are contiguos

    if(gait_generator_->isLiftOff(foot_names[i]))
        des_joint_positions_.segment(idx,3) = current_joint_positions.segment(idx,3);

    J_foot_ = J_.block<3,3>(0,idx);
    J_foot_transp_ = J_foot_.transpose();
    double damp = std::exp(-4.0/determinant_max_*std::abs(J_foot_.determinant()))*damp_max_;
    J_foot_inv_ = J_foot_transp_ * (J_foot_ * J_foot_transp_ + damp*damp * I_).inverse();

    x_err_ = desired_foot_positions_[foot_names[i]] - world_T_base_.translation();

    des_joint_velocities_.segment(idx,3) = J_foot_inv_ * (clik_gain_ * x_err_);

    des_joint_positions_.segment(idx,3) = des_joint_velocities_.segment(idx,3) * period + des_joint_positions_.segment(idx,3);
  }

  // Check if the desired positions and velocities are valid and clamp them
  robot_model_->clampJointPositions(des_joint_positions_);
  robot_model_->clampJointVelocities(des_joint_velocities_);

  return true;
}

void LegsKinematics::setClikGain(const double& clik_gain)
{
  if(clik_gain < 0.0) // Check if it is ok
      ROS_WARN_NAMED(CLASS_NAME,"CLIK gain has to be positive!");
  else
  {
     clik_gain_ = clik_gain;
     ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set CLIK gain at "<< clik_gain);
  }
}

double LegsKinematics::getClikGain()
{
  return clik_gain_;
}

const Eigen::VectorXd& LegsKinematics::getDesiredJointPositions()
{
  return des_joint_positions_;
}

const Eigen::VectorXd& LegsKinematics::getDesiredJointVelocities()
{
  return des_joint_velocities_;
}

bool LegsKinematics::setJointHomePositions(Eigen::VectorXd& qhome)
{
  // Check if qhome is between qmin and qmax
  if(!robot_model_->checkJointLimits(qhome))
  {
      ROS_WARN_NAMED(CLASS_NAME,"Can not set qhome, joint limits violated!");
      return false;
  }
  else
    qhome_ = qhome;

  return true;
}

const Eigen::VectorXd& LegsKinematics::getJointHomePositions()
{
  return qhome_;
}

void LegsKinematics::reset()
{
  des_joint_velocities_.setZero();
  robot_model_->getJointPosition(des_joint_positions_);

  auto foot_names = gait_generator_->getFootNames();
  for(unsigned int i=0; i<foot_names.size(); i++)
      desired_foot_positions_[foot_names[i]] = robot_model_->getFootPositionInWorld(foot_names[i]);
}

void LegsKinematics::setAdaptiveDamping(const double &damp_max, const double &determinant_max)
{
    assert(damp_max>=0.0);
    damp_max_ = damp_max;
    assert(determinant_max>=0.0);
    determinant_max_ = determinant_max;
}

} // namespace
