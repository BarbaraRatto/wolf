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
  computeComPositionReference();

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/com_velocity_ref",com_velocity_ref_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/com_position_ref",com_position_ref_);
}

void ComPlanner::computeSupportPolygonCenter()
{
  auto foot_positions = robot_model_->getFeetPositionInWorld();
  auto foot_names = foothold_planner_->getFootNames();
  support_polygon_center_.setZero();
  for(unsigned int i = 0; i<foot_names.size(); i++)
    support_polygon_center_ = support_polygon_center_ + foot_positions[foot_names[i]];

  support_polygon_center_ = support_polygon_center_/N_LEGS;
}

void ComPlanner::computeComPositionReference()
{
  computeSupportPolygonCenter();
  com_position_ref_ << support_polygon_center_(0), support_polygon_center_(1), foothold_planner_->getBaseHeight();
}

void ComPlanner::computeComVelocityReference()
{

  //https://kodlab.seas.upenn.edu/uploads/Kod/Schwind95.pdf
  //vcom = 2*X_piede/T

  base_velocity_ = 0.5 * foothold_planner_->getBaseLinearVelocityReference();

  com_velocity_ref_.setZero();
  com_velocity_ref_ = terrain_estimator_->getTerrainOrientationWorld().transpose() * base_velocity_; // getPose(): world_T_terrain
  //com_velocity_ref_.z() = 0.0; // No height reference, only damping

  if(foothold_planner_->getGaitType() == Gait::CRAWL) // With the crawl the robot moves approx half the speed
    com_velocity_ref_ = 0.5 * com_velocity_ref_;
}

void ComPlanner::update(double /*dt*/)
{

   computeComVelocityReference();

   // Update the support polygon everytime there is a touchdown
   //if(foothold_planner_->isAnyFootInTouchDown())
   //  update_ = true;

   // If all feet in stance then update the support polygon center
   if (foothold_planner_->areAllFeetInStance())
   {
     //if(update_)
     //{
       computeComPositionReference();
       update_ = false;
     //}
   }
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
