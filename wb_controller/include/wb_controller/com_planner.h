#ifndef COM_PLANNER_H
#define COM_PLANNER_H

#include <memory>
#include <Eigen/Core>
#include <wb_controller/state_estimator.h>
#include <wb_controller/footholds_planner.h>
#include <wb_controller/terrain_estimator.h>

namespace wb_controller
{

class ComPlanner
{

public:

  const std::string CLASS_NAME = "ComPlanner";

  /**
   * @brief Shared pointer to ComPlanner
   */
  typedef std::shared_ptr<ComPlanner> Ptr;

  /**
   * @brief Shared pointer to const ComPlanner
   */
  typedef std::shared_ptr<const ComPlanner> ConstPtr;

  ComPlanner(QuadrupedRobot::Ptr robot_model, FootholdsPlanner::Ptr foothold_planner, TerrainEstimator::Ptr terrain_estimator);

  void update(double dt);

  const Eigen::Vector3d& getComVelocity() const;

  const Eigen::Vector3d& getComPosition() const;

private:

  double c_, c_filt_, c_ref_;
  std::vector<std::string> feet_order_;
  QuadrupedRobot::Ptr robot_model_;
  FootholdsPlanner::Ptr foothold_planner_;
  TerrainEstimator::Ptr terrain_estimator_;
  Eigen::Vector3d com_velocity_ref_;
  Eigen::Vector3d com_position_ref_;
  Eigen::Vector3d base_X_com_;

  Eigen::Vector2d p0_;
  Eigen::Vector2d p1_;
  Eigen::Vector2d p2_;
  Eigen::Vector3d base_velocity_;
  Eigen::Vector2d base_velocity_xy_;
  Eigen::Vector2d base_velocity_xy_versor_;
  Eigen::Vector2d p_int_;
  Eigen::Vector2d p_int_versor_;


};


} // namespace

#endif
