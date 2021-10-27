#include <wb_controller/com_planner.h>

using namespace rt_logger;

namespace wb_controller {

ComPlanner::ComPlanner(QuadrupedRobot::Ptr robot_model, FootholdsPlanner::Ptr foothold_planner, TerrainEstimator::Ptr terrain_estimator)
{

  robot_model_ = robot_model;
  foothold_planner_ = foothold_planner;
  terrain_estimator_ = terrain_estimator;

  com_velocity_ref_.setZero();
  com_position_ref_.setZero();

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/com_velocity_ref",com_velocity_ref_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/com_position_ref",com_position_ref_);
}

void ComPlanner::update(double /*dt*/)
{

  base_velocity_ = foothold_planner_->getBaseLinearVelocityReference();

  com_velocity_ref_.setZero();
  com_velocity_ref_ = terrain_estimator_->getTerrainOrientationWorld().transpose() * base_velocity_; // getPose(): world_T_terrain
  //com_velocity_ref_.z() = 0.0; // No height reference, only damping

  if(foothold_planner_->getGaitType() == Gait::CRAWL) // With the crawl the robot moves approx half the speed
    com_velocity_ref_ = 0.5 * com_velocity_ref_;
}

const Eigen::Vector3d &ComPlanner::getComVelocity() const
{
  return com_velocity_ref_;
}

const Eigen::Vector3d &ComPlanner::getComPosition() const
{
  return com_position_ref_;
}

}; // namespace
