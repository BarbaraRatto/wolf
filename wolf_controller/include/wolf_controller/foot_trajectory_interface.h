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

  TrajectoryInterface()
    :trajectory_id(_id++)
  {
    pose_reference_ = initial_pose_ = pose_ = Eigen::Affine3d::Identity();
    twist_reference_.setZero();
    twist_.setZero();
    swing_frequency_ = 0.0;
    time_ = 0.0;
    length_ = 0.0;
    heading_ = 0.0;
    heading_rate_ = 0.0;
    height_ = 0.0;
    world_R_terrain_.setIdentity();

    trajectory_finished_ = true;
#ifdef DEBUG
    rt_logger::RtLogger::getLogger().addPublisher(TOPIC(position_reference_)+_legs_prefix[trajectory_id],position_reference_);
    rt_logger::RtLogger::getLogger().addPublisher(TOPIC(velocity_reference_)+_legs_prefix[trajectory_id],velocity_reference_);
#endif
  }

  const Eigen::Affine3d& getReference()
  {
    return pose_reference_;
  }

  const Eigen::Vector6d& getReferenceDot()
  {
    return twist_reference_;
  }

  const Eigen::Affine3d& getInitialPose()
  {
    return initial_pose_;
  }

  void setInitialPose(const Eigen::Affine3d& initial_pose)
  {
#ifdef OPEN_LOOP_TRAJECTORY
    // Open loop trajectory
    initial_pose_ = pose_reference_;
#else
    // Closed loop trajectory: to be used if the tracking is good
    initial_pose_ = initial_pose;
    pose_reference_ = initial_pose;
#endif
  }

  bool isFinished()
  {
    return trajectory_finished_;
  }

  void start()
  {
    time_ = 0.0;
    twist_reference_.setZero();
    trajectory_finished_ = false;
  }

  void stop()
  {
    twist_reference_.setZero();
  }

  void setStepHeading(const double& heading)
  {
    heading_ = heading;
  }

  void setStepHeadingRate(const double& heading_rate)
  {
    heading_rate_ = heading_rate;
  }

  void setTerrainRotation(const Eigen::Matrix3d& world_R_terrain)
  {
    world_R_terrain_ = world_R_terrain;
  }

  void setStepHeight(const double& height)
  {
    height_ = height;
  }

  void setStepLength(const double& length)
  {
    length_ = length;
  }

  const double& getStepLength() const
  {
    return length_;
  }

  const double& getStepHeight() const
  {
    return height_;
  }

  const double& getStepHeading() const
  {
    return heading_;
  }

  const double& getStepHeadingRate() const
  {
    return heading_rate_;
  }

  const Eigen::Matrix3d& getTerrainRotation() const
  {
    return world_R_terrain_;
  }

  void setSwingFrequency(const double& swing_frequency)
  {
    if(swing_frequency >= 0.0)
      swing_frequency_ = swing_frequency;
    else
      ROS_WARN_NAMED(CLASS_NAME,"Swing frequency has to be positive definite!");
  }

  double getSwingFrequency()
  {
    return swing_frequency_;
  }

  void update(const double& period)
  {
    time_ += period;

    pose_reference_  = trajectoryFunction(time_);
    twist_reference_ = trajectoryFunctionDot(time_);

    position_reference_ = pose_reference_.translation();
    velocity_reference_ = twist_reference_.head(3);

    if(swing_frequency_*time_>=1.0)
      trajectory_finished_ = true;
  }

  double getCompletion()
  {
    return swing_frequency_*time_;
  }

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
