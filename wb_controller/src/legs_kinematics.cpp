#include "wb_controller/legs_kinematics.h"

namespace wb_controller {

LegsKinematics::LegsKinematics(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model)
  : base_height_control_active_(false), clik_gain_(1.0)
{

  assert(gait_generator);
  gait_generator_ = gait_generator;
  assert(robot_model);
  robot_model_ = robot_model;

  // Set home position, qmin and qmax defined in the srdf
  // Initial values
  robot_model_->getXBotModel()->getRobotState("home", qhome_);
  robot_model_->getXBotModel()->setJointPosition(qhome_);

  robot_model_->getXBotModel()->getJointLimits(qmin_, qmax_);

  des_joint_positions_.resize(qhome_.size());
  des_joint_velocities_.resize(qhome_.size());

  // Initializations
  reset();
  //des_joint_positions_.fill(0.0);
  //des_joint_velocities_.fill(0.0);
  //qstance_ = qswing_ = qhome_;
}

bool LegsKinematics::update(const double& period, const Eigen::VectorXd& current_joint_positions)
{

  robot_model_->getXBotModel()->getFloatingBasePose(tmp_affine3d_); // This should have been already updated by the state estimator
  base_height_ = tmp_affine3d_.translation()(2);
  robot_model_->getXBotModel()->getFloatingBaseTwist(tmp_vector6d_);
  base_height_dot_ = tmp_vector6d_(2);


  des_joint_positions_ = current_joint_positions;

  double delta_z = des_base_height_ - base_height_;
  double delta_z_dot = - base_height_dot_;

  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& leg_names = robot_model_->getLegNames();

  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    robot_model_->getXBotModel()->getJacobian(foot_names[i],J_);
    robot_model_->getXBotModel()->getPose(foot_names[i],world_T_foot_);

    int idx = robot_model_->getLegJointsIds(leg_names[i])[0]; // NOTE: take the first idx, hopefully the leg joints are contiguos

    J_foot_ = J_.block<3,3>(0,idx);

    if(gait_generator_->isSwinging(foot_names[i]))
    {
      // At the first cycle of swing, set the des joints position at the current measured joints position
      if(gait_generator_->isLiftOff(foot_names[i]))
        qswing_.segment(idx,3) = current_joint_positions.segment(idx,3);

      x_err_ = gait_generator_->getReference(foot_names[i]).translation() - world_T_foot_.translation();

      xdot_swing_ff_ = gait_generator_->getReferenceDot(foot_names[i]).segment(0,3);

      des_joint_velocities_.segment(idx,3) = J_foot_.inverse() * (xdot_swing_ff_ + clik_gain_ * x_err_);

      qswing_.segment(idx,3) = des_joint_velocities_.segment(idx,3) * period + qswing_.segment(idx,3);

      des_joint_positions_.segment(idx,3) = qswing_.segment(idx,3);
    }
    else
    {
      // At the first cycle of stance, set the des joints position at the current homing position
      //if(gait_generator_->isTouchDown(foot_names_[i]))
      //   des_joint_positions_.segment(idx,3) = qhome_.segment(idx,3);

      if(base_height_control_active_)
      {
        x_err_ << 0, 0, -delta_z;
        x_err_dot_ << 0, 0, -delta_z_dot;
      }
      else
      {
        x_err_ << 0, 0, 0;
        x_err_dot_ << 0, 0, 0;
      }

      qstance_.segment(idx,3) = J_foot_.inverse() * (xdot_stance_ff_ +  clik_gain_ * x_err_ + 2*std::sqrt(clik_gain_) * x_err_dot_) * period + qstance_.segment(idx,3);

      des_joint_velocities_.segment(idx,3).fill(0.0);  // Don't generate velocities for the feet in stance
      des_joint_positions_.segment(idx,3) = qstance_.segment(idx,3);
    }
  }

  // Check if the desired positions are valid and clamp them
  jointLimitsCheck(des_joint_positions_,qmin_,qmax_);

  return true;
}

void LegsKinematics::setClikGain(const double& clik_gain)
{
  if(clik_gain < 0.0) // Check if it is ok
      ROS_WARN_NAMED(CLASS_NAME,"Clik gain has to be positive!");
  else
     clik_gain_ = clik_gain;
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
  if(jointLimitsCheck(qhome,qmin_,qmax_))
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
  x_err_dot_.setZero();
  robot_model_->getXBotModel()->getJointPosition(des_joint_positions_);
  qstance_ = des_joint_positions_;
  qswing_ = des_joint_positions_;
  robot_model_->getXBotModel()->getFloatingBasePose(tmp_affine3d_); // This should have been already updated by the state estimator
  des_base_height_ = base_height_ = tmp_affine3d_.translation()(2);
  xdot_stance_ff_.setZero();
  xdot_swing_ff_.setZero();
}

void LegsKinematics::setJointLimits(const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin)
{
   qmax_ = qmax;
   qmin_ = qmin;
}

bool LegsKinematics::jointLimitsCheck(Eigen::VectorXd& q, const Eigen::VectorXd& qmin, const Eigen::VectorXd& qmax)
{
    assert(q.size() == qmin.size());
    assert(qmin.size() == qmax.size());
    bool violated_limits = false;
    for(unsigned int i=0;i<q.size();i++)
    {
        if(q(i)<qmin(i))
        {
            q(i) = qmin(i);
            //ROS_WARN_STREAM_NAMED(CLASS_NAME,"Joint("<<_dof_names[i]<<") violates the minimum limit of "<<qmin(i));
            violated_limits = true;
        }
        if(q(i)>qmax(i))
        {
            q(i) = qmax(i);
            //ROS_WARN_STREAM_NAMED(CLASS_NAME,"Joint("<<_dof_names[i]<<") violates the maximum limit of "<<qmax(i));
            violated_limits = true;
        }
    }
    return violated_limits;
}

void LegsKinematics::setDesiredBaseHeight(const double& des_base_height)
{
  des_base_height_ = des_base_height;
}

void LegsKinematics::setFeedForwardStanceDot(const Eigen::Vector3d& xdot_stance_ff)
{
  xdot_stance_ff_ = xdot_stance_ff;
}

void LegsKinematics::setFeedForwardSwingDot(const Eigen::Vector3d& xdot_swing_ff)
{
  xdot_swing_ff_ = xdot_swing_ff;
}

void LegsKinematics::activateBaseHeightControl()
{
  base_height_control_active_ = true;
}

void LegsKinematics::deactivateBaseHeightControl()
{
  base_height_control_active_ = false;
}

void LegsKinematics::toggleBaseHeightControl()
{
  base_height_control_active_=!base_height_control_active_;
}

bool LegsKinematics::isBaseHeightControlActive()
{
  if(base_height_control_active_)
    return true;
  else
    return false;
}

} // namespace
