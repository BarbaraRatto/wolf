#include <wolf_controller/foot_trajectory_ellipse.h>
#include <wolf_controller/geometry.h>

using namespace wolf_controller;

Ellipse::Ellipse()
{
  xyz_ = xyz_rotated_ = xyz_dot_ = Eigen::Vector3d::Zero();
  terrain_R_swing_ = terrain_R_world_ = world_Rz_swing_ = Sz_ = Ear_ = Eigen::Matrix3d::Zero();
}

const Eigen::Affine3d& Ellipse::trajectoryFunction(const double& time)
{
  xyz_(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
  xyz_(1) = 0.0;
  xyz_(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

  double c = std::cos(heading_);
  double s = std::sin(heading_);

  world_Rz_swing_(0,0) = c;
  world_Rz_swing_(0,1) = -s;
  world_Rz_swing_(1,0) = s;
  world_Rz_swing_(1,1) = c;
  world_Rz_swing_(2,2) = 1;

  terrain_R_world_.noalias() = world_R_terrain_.transpose();

  terrain_R_swing_.noalias() = terrain_R_world_ * world_Rz_swing_;

  xyz_rotated_.noalias() = terrain_R_swing_ * xyz_;

  pose_.translation() = initial_pose_.translation() + xyz_rotated_;

  return pose_;
}

const Eigen::Vector6d& Ellipse::trajectoryFunctionDot(const double& time)
{

  xyz_dot_(0) = M_PI * swing_frequency_ * length_/2 * std::sin(M_PI * (swing_frequency_ * time));
  xyz_dot_(1) = 0.0;
  xyz_dot_(2) = M_PI * swing_frequency_ * height_ * std::cos(M_PI * (swing_frequency_ * time));

  rpy_rates_.setZero();
  omegas_.setZero();
  rpy_.setZero();

  rpy_(2) = heading_;
  rpy_rates_(2) = heading_rate_;

  rpyToEarWorld(rpy_,Ear_);

  omegas_ = Ear_ * rpy_rates_;

  Sz_(0,1) = -omegas_(2);
  Sz_(1,0) = -omegas_(2);

  twist_.setZero(); // No angular velocities
  // We update the terrain frame each time all the feet are in touchdown, therefore
  // the terrain estimation does not contribute to the feet velocities but it just rotates them
  twist_.head(3) = terrain_R_world_ * (Sz_ * world_Rz_swing_ * xyz_ + world_Rz_swing_ * xyz_dot_);

  return twist_;
}
