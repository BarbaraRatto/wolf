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

  QuadrupedRobot::Ptr robot_model_;
  FootholdsPlanner::Ptr foothold_planner_;
  TerrainEstimator::Ptr terrain_estimator_;
  Eigen::Vector3d com_velocity_ref_;
  Eigen::Vector3d com_position_ref_;
  Eigen::Vector3d base_velocity_;

};


} // namespace

#endif
