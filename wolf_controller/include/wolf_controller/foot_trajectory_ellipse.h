#ifndef FOOT_TRAJECTORY_ELLIPSE_H
#define FOOT_TRAJECTORY_ELLIPSE_H

#include <wolf_controller/foot_trajectory_interface.h>

namespace wolf_controller
{

class Ellipse : public TrajectoryInterface
{

public:

  const std::string CLASS_NAME = "Ellipse";

  Ellipse();

protected:

  virtual const Eigen::Affine3d& trajectoryFunction(const double& time);

  virtual const Eigen::Vector6d& trajectoryFunctionDot(const double& time);

private:

  Eigen::Vector3d xyz_;
  Eigen::Vector3d xyz_dot_;
  Eigen::Vector3d xyz_rotated_;

  Eigen::Matrix3d Sz_;
  Eigen::Matrix3d Ear_;

  Eigen::Vector3d rpy_;
  Eigen::Vector3d rpy_rates_;
  Eigen::Vector3d omegas_;

  Eigen::Matrix3d world_Rz_swing_;
  Eigen::Matrix3d terrain_R_world_;
  Eigen::Matrix3d terrain_R_swing_;
};

} // namespace

#endif
