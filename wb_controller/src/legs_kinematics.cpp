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

bool LegsKinematics::update()
{

  des_joint_velocities_.fill(0.0);

  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  const std::vector<std::string>& leg_names = robot_model_->getLegNames();

  for(unsigned int i = 0; i<foot_names.size(); i++)
  {

    if(gait_generator_->isInStance(foot_names[i]))
    {

        int idx = robot_model_->getLimbJointsIds(leg_names[i])[0]; // NOTE: take the first idx because the leg joints are contiguos

        robot_model_->getPose(robot_model_->getBaseLinkName(),world_T_base_);
        robot_model_->getPose(foot_names[i],world_T_foot_);

        world_adj_ = world_T_foot_.translation() - world_T_base_.translation();
        base_adj_ =  world_T_base_.rotation().transpose() * world_adj_;

        q_.segment(idx,3) = qhome_.segment(idx,3);


        while(true)
        {
            robot_model_->getPose(q_,foot_names[i],robot_model_->getBaseLinkName(),tmp_affine3d_);
            robot_model_->getJacobian(q_,foot_names[i],robot_model_->getBaseLinkName(),J_);

            J_foot_ = J_.block<3,3>(0,idx);
            J_foot_inv_ = J_foot_.inverse();

            tmp_vector3d_ = J_foot_inv_ * (base_adj_ - tmp_affine3d_.translation());
            q_.segment(idx,3) = tmp_vector3d_ + q_.segment(idx,3);

            if(tmp_vector3d_.norm() < EPS)
                break;
        }
        des_joint_positions_.segment(idx,3) = q_.segment(idx,3);
    }
  }

  // Check if the desired positions and velocities are valid and clamp them
  //robot_model_->clampJointPositions(des_joint_positions_);
  //robot_model_->clampJointVelocities(des_joint_velocities_);

  return true;
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
