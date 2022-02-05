#include <wolf_controller/foot_trajectory_interface.h>

using namespace wolf_controller;

TrajectoryInterface::TrajectoryInterface()
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

const Eigen::Affine3d& TrajectoryInterface::getReference()
{
  return pose_reference_;
}

const Eigen::Vector6d& TrajectoryInterface::getReferenceDot()
{
  return twist_reference_;
}

const Eigen::Affine3d& TrajectoryInterface::getInitialPose()
{
  return initial_pose_;
}

void TrajectoryInterface::setInitialPose(const Eigen::Affine3d& initial_pose)
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

bool TrajectoryInterface::isFinished()
{
  return trajectory_finished_;
}

void TrajectoryInterface::start()
{
  time_ = 0.0;
  twist_reference_.setZero();
  trajectory_finished_ = false;
}

void TrajectoryInterface::stop()
{
  twist_reference_.setZero();
}

void TrajectoryInterface::setStepHeading(const double& heading)
{
  heading_ = heading;
}

void TrajectoryInterface::setStepHeadingRate(const double& heading_rate)
{
  heading_rate_ = heading_rate;
}

void TrajectoryInterface::setTerrainRotation(const Eigen::Matrix3d& world_R_terrain)
{
  world_R_terrain_ = world_R_terrain;
}

void TrajectoryInterface::setStepHeight(const double& height)
{
  height_ = height;
}

void TrajectoryInterface::setStepLength(const double& length)
{
  length_ = length;
}

const double& TrajectoryInterface::getStepLength() const
{
  return length_;
}

const double& TrajectoryInterface::getStepHeight() const
{
  return height_;
}

const double& TrajectoryInterface::getStepHeading() const
{
  return heading_;
}

const double& TrajectoryInterface::getStepHeadingRate() const
{
  return heading_rate_;
}

const Eigen::Matrix3d& TrajectoryInterface::getTerrainRotation() const
{
  return world_R_terrain_;
}

void TrajectoryInterface::setSwingFrequency(const double& swing_frequency)
{
  if(swing_frequency >= 0.0)
    swing_frequency_ = swing_frequency;
  else
    ROS_WARN_NAMED(CLASS_NAME,"Swing frequency has to be positive definite!");
}

double TrajectoryInterface::getSwingFrequency()
{
  return swing_frequency_;
}

void TrajectoryInterface::update(const double& period)
{
  time_ += period;

  pose_reference_  = trajectoryFunction(time_);
  twist_reference_ = trajectoryFunctionDot(time_);

  position_reference_ = pose_reference_.translation();
  velocity_reference_ = twist_reference_.head(3);

  if(swing_frequency_*time_>=1.0)
    trajectory_finished_ = true;
}

double TrajectoryInterface::getCompletion()
{
  return swing_frequency_*time_;
}

