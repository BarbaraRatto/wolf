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

  support_polygon_edges_.resize(N_LEGS);

  update_ = true;
  computeComPositionReference();

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/com_velocity_ref",com_velocity_ref_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/com_position_ref",com_position_ref_);
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
  //if(foothold_planner_->isAnyFootInTouchDown())
  //  update_ = true;

  // If all feet in stance then update the support polygon center
  if (foothold_planner_->areAllFeetInStance())
  {
    //if(update_)
    //{
      computeSupportPolygonCenter();
      update_ = false;
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

  if(foothold_planner_->getGaitType() == Gait::TROT)
    com_velocity_ref_ = 0.5 * com_velocity_ref_;
  else if (foothold_planner_->getGaitType() == Gait::CRAWL)
    com_velocity_ref_ = 0.25 * com_velocity_ref_;
  else
    com_velocity_ref_ = 1.0 * com_velocity_ref_;
}

void ComPlanner::update()
{
  computeComVelocityReference();
  computeComPositionReference();

  // Check if we are out of the bounding box
  double max_x = 0.0;
  double max_y = 0.0;
  double min_x = 0.0;
  double min_y = 0.0;
  for(unsigned int i=0;i<N_LEGS;i++)
  {
    if (support_polygon_edges_[i].x() >= max_x)
      max_x = support_polygon_edges_[i].x();
    if (support_polygon_edges_[i].y() >= max_y)
      max_y = support_polygon_edges_[i].y();
    if (support_polygon_edges_[i].x() <= min_x)
      min_x = support_polygon_edges_[i].x();
    if (support_polygon_edges_[i].y() <= min_y)
      min_y = support_polygon_edges_[i].y();
  }
  robot_model_->getCOM(com_);
  if(com_.x() > max_x || com_.y() > max_y || com_.x() < min_x || com_.y() < min_y)
  {
    robot_model_->setState(QuadrupedRobot::ANOMALY);
    ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"CoM is outside bounding box!");
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
