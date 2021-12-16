#include "wb_controller/legs_kinematics.h"

namespace wb_controller {

LegsKinematics::LegsKinematics(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model, TerrainEstimator::Ptr terrain_estimator)
{

  assert(gait_generator);
  gait_generator_ = gait_generator;
  assert(robot_model);
  robot_model_ = robot_model;
  assert(terrain_estimator);
  terrain_estimator_ = terrain_estimator;

  // Set home position, qmin and qmax defined in the srdf
  // Initial values
  qhome_ = q_ = robot_model_->getJointHomePositions();

  des_joint_positions_.resize(qhome_.size());
  des_joint_velocities_.resize(qhome_.size());

  reset();
}

bool LegsKinematics::update(const Eigen::VectorXd& current_joint_positions)
{

  des_joint_velocities_.fill(0.0);

  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& leg_names = robot_model_->getLegNames();

  world_T_base_ = robot_model_->getBasePoseInWorld();

  des_joint_positions_ = q_ = current_joint_positions;

  for(unsigned int i = 0; i<foot_names.size(); i++)
  {

    int idx = robot_model_->getLimbJointsIds(leg_names[i])[0]; // NOTE: take the first idx because the leg joints are contiguos

    robot_model_->getPose(foot_names[i],world_T_foot_);

    if(gait_generator_->isInStance(foot_names[i]))
    {
      world_adj_ = world_T_foot_.translation() - world_T_base_.translation();
      base_adj_ =  world_T_base_.linear().transpose() * world_adj_;

      while(true)
      {
          robot_model_->getPose(q_,foot_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_);
          robot_model_->getJacobian(q_,foot_names[i],robot_model_->getBaseLinkName(),J_);

          J_foot_ = J_.block<3,3>(0,idx);
          J_foot_inv_ = J_foot_.inverse();

          tmp_vector3d_ = J_foot_inv_ * (base_adj_ - tmp_affine3d_.translation());
          q_.segment(idx,3) = tmp_vector3d_ + q_.segment(idx,3);

          if(tmp_vector3d_.norm() < EPS)
          {
              //des_joint_positions_.segment(idx,3) = q_.segment(idx,3);
              break;
          }
      }
    }
    else
    {

      //world_adj_ = gait_generator_->getReference(foot_names[i]).translation();
      //base_adj_ =  world_T_base_.rotation().transpose() * world_adj_;
      //des_joint_positions_.segment(idx,3) = computeIk(idx,foot_names[i],current_joint_positions,base_adj_);

      world_adj_ = gait_generator_->getReference(foot_names[i]).translation();
      base_adj_ =  world_T_base_.inverse() * world_adj_;

      while(true)
      {
          robot_model_->getPose(q_,foot_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_);
          robot_model_->getJacobian(q_,foot_names[i],robot_model_->getBaseLinkName(),J_);

          J_foot_ = J_.block<3,3>(0,idx);
          J_foot_inv_ = J_foot_.inverse();

          tmp_vector3d_ = J_foot_inv_ * (base_adj_ - tmp_affine3d_.translation());
          q_.segment(idx,3) = tmp_vector3d_ + q_.segment(idx,3);

          if(tmp_vector3d_.norm() < EPS)
          {
              //des_joint_positions_.segment(idx,3) = q_.segment(idx,3);
              break;
          }
      }
    }
  }

  des_joint_positions_ = q_;

  // Check if the desired positions and velocities are valid and clamp them
  robot_model_->clampJointPositions(des_joint_positions_);
  //robot_model_->clampJointVelocities(des_joint_velocities_);

  return true;
}

const Eigen::VectorXd& LegsKinematics::computeIk(const unsigned int& idx, const std::string& foot_name, const Eigen::VectorXd& q0, const Eigen::Vector3d& ref)
{
  q_ = q0;
  while(true)
  {
      robot_model_->getPose(q_,foot_name,robot_model_->getBaseLinkName(),tmp_affine3d_);
      robot_model_->getJacobian(q_,foot_name,robot_model_->getBaseLinkName(),J_);

      J_foot_ = J_.block<3,3>(0,idx);
      J_foot_inv_ = J_foot_.inverse();

      tmp_vector3d_ = J_foot_inv_ * (ref - tmp_affine3d_.translation());
      q_.segment(idx,3) = tmp_vector3d_ + q_.segment(idx,3);

      if(tmp_vector3d_.norm() < EPS)
      {
          break;
      }
  }
  return q_;
}

const Eigen::VectorXd& LegsKinematics::getDesiredJointPositions()
{
  return des_joint_positions_;
}

const Eigen::VectorXd& LegsKinematics::getDesiredJointVelocities()
{
  return des_joint_velocities_;
}

void LegsKinematics::reset()
{
  des_joint_velocities_.setZero();
  robot_model_->getJointPosition(des_joint_positions_);
}

} // namespace
