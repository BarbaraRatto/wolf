/**
 * @file com_planner.cpp
 * @author Gennaro Raiola
 * @date 1 November, 2021
 * @brief This file contains the ComPlanner used to generate the com position and velocity references
 */

#include <wolf_controller/com_planner.h>

using namespace rt_logger;

namespace wolf_controller {

ComPlanner::ComPlanner(QuadrupedRobot::Ptr robot_model, FootholdsPlanner::Ptr foothold_planner, TerrainEstimator::Ptr terrain_estimator)
{

  robot_model_ = robot_model;
  foothold_planner_ = foothold_planner;
  terrain_estimator_ = terrain_estimator;

  com_velocity_ref_.setZero();
  com_position_ref_.setZero();

  support_polygon_edges_.resize(N_LEGS);

  update_ = true;

  computeComPositionReference();
}

void ComPlanner::computeSupportPolygonCenter()
{
  // Note: the com position reference has to be defined wrt world because
  // the com task is wrt world
  auto foot_positions = robot_model_->getFeetPositionInWorld();
  auto foot_names = foothold_planner_->getFootNames();
  support_polygon_center_.setZero();
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    support_polygon_center_ = support_polygon_center_ + foot_positions[foot_names[i]];
    support_polygon_edges_[i] = foot_positions[foot_names[i]];
  }

  support_polygon_center_ = support_polygon_center_/N_LEGS;
}

void ComPlanner::computeComPositionReference()
{
  // Update the support polygon everytime there is a touchdown
  // or if the robot is standing up or down (because if we
  // are using the estimated_z the com reference is calculated wrt the base which is
  // moving up/down)
  //if(foothold_planner_->isAnyFootInTouchDown()
  //   || robot_model_->getState() == QuadrupedRobot::STANDING_UP
  //   || robot_model_->getState() == QuadrupedRobot::STANDING_DOWN
  //   )
  //  update_ = true;

  // If all feet in stance then update the support polygon center
  if (foothold_planner_->areAllFeetInStance())
  {
    //if(update_)
    //{
      computeSupportPolygonCenter();
     // update_ = false;
    //}
  }

  com_position_ref_ << support_polygon_center_(0), support_polygon_center_(1), foothold_planner_->getBaseHeight();
}

void ComPlanner::computeComVelocityReference()
{

  //https://kodlab.seas.upenn.edu/uploads/Kod/Schwind95.pdf
  //vcom = 2*X_piede/T

  base_velocity_ = foothold_planner_->getBaseLinearVelocityReference();

  com_velocity_ref_.setZero();
  com_velocity_ref_ = terrain_estimator_->getTerrainOrientationWorld().transpose() * base_velocity_; // getPose(): world_T_terrain
  //com_velocity_ref_.z() = 0.0; // No height reference, only damping

  com_velocity_ref_ = 1.0/foothold_planner_->getVelocityFactor() * com_velocity_ref_;

}

void ComPlanner::update()
{
  computeComVelocityReference();
  computeComPositionReference();
}

const Eigen::Vector3d &ComPlanner::getComVelocity() const
{
  return com_velocity_ref_;
}

const Eigen::Vector3d &ComPlanner::getComPosition() const
{
  return com_position_ref_;
}

void ComPlanner::resetVelocities()
{
  com_velocity_ref_.setZero();
}

}; // namespace
