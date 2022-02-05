#ifndef FOOT_TRAJECTORY_INTERFACE_H
#define FOOT_TRAJECTORY_INTERFACE_H

#include <atomic>
#include <Eigen/Dense>
#include <wolf_controller/utils.h>

namespace wolf_controller
{

class TrajectoryInterface
{

public:

  const std::string CLASS_NAME = "TrajectoryInterface";

  TrajectoryInterface();

  const Eigen::Affine3d& getReference();

  const Eigen::Vector6d& getReferenceDot();

  const Eigen::Affine3d& getInitialPose();

  void setInitialPose(const Eigen::Affine3d& initial_pose);

  bool isFinished();

  void start();

  void stop();

  void setStepHeading(const double& heading);

  void setStepHeadingRate(const double& heading_rate);

  void setTerrainRotation(const Eigen::Matrix3d& world_R_terrain);

  void setStepHeight(const double& height);

  void setStepLength(const double& length);

  const double& getStepLength() const;

  const double& getStepHeight() const;

  const double& getStepHeading() const;

  const double& getStepHeadingRate() const;

  const Eigen::Matrix3d& getTerrainRotation() const;

  void setSwingFrequency(const double& swing_frequency);

  double getSwingFrequency();

  void update(const double& period);

  double getCompletion();

protected:

  inline static int _id = 0;
  int trajectory_id;

  /** @brief Internal time variable */
  double time_;
  /** @brief Swing frequency */
  double swing_frequency_;
  /** @brief Step length */
  double length_;
  /** @brief Heading represents the yaw rotation between the swing frame and world frame */
  double heading_;
  /** @brief Heading rate represents the derivate of the yaw rotation */
  double heading_rate_;
  /** @brief Step height */
  double height_;
  /** @brief Rotation between terrain frame and world, it is used to adapt
      the swing trajectory to align with the terrain */
  Eigen::Matrix3d world_R_terrain_;
  /** @brief Check if the trajectory cycle is ended */
  bool trajectory_finished_;
  /** @brief Trajectory pose reference output */
  Eigen::Affine3d pose_reference_;
  /** @brief Trajectory twist reference output */
  Eigen::Vector6d twist_reference_;
  /** @brief Trajectory position reference output */
  Eigen::Vector3d position_reference_;
  /** @brief Trajectory velocity reference output */
  Eigen::Vector3d velocity_reference_;
  /** @brief Initial pose for the trajectory generation */
  Eigen::Affine3d initial_pose_;
  /** @brief Internal pose of the trajectory */
  Eigen::Affine3d pose_;
  /** @brief Internal twist of the trajectory */
  Eigen::Vector6d twist_;

  virtual const Eigen::Affine3d& trajectoryFunction(const double& time) = 0;
  virtual const Eigen::Vector6d& trajectoryFunctionDot(const double& time) = 0;
};

} // namespace

#endif
