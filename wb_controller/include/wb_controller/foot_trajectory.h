#ifndef FOOT_TRAJECTORY_H
#define FOOT_TRAJECTORY_H

#include <atomic>
#include <Eigen/Dense>

namespace wb_controller
{

class TrajectoryInterface
{

public:

  TrajectoryInterface()
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

    trajectory_finished_ = true;
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

  void standBy()
  {
    twist_reference_.setZero();
  }

  void setStepHeadingRate(const double& heading_rate)
  {
    heading_rate_ = heading_rate;
  }

  void setStepLength(const double& length)
  {
    length_ = length;
  }

  void setStepHeading(const double& heading)
  {
    heading_ = heading;
  }

  void setStepHeight(const double& height)
  {
    height_ = height;
  }

  double getStepLength()
  {
    return length_;
  }

  double getStepHeading()
  {
    return heading_;
  }

  double getStepHeadingRate()
  {
    return heading_rate_;
  }

  double getStepHeight()
  {
    return height_;
  }

  void setSwingFrequency(const double& swing_frequency)
  {
    if(swing_frequency >= 0.0)
      swing_frequency_ = swing_frequency;
    else
      ROS_WARN("Swing frequency has to be positive definite!");
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

    if(swing_frequency_*time_>=1.0)
      trajectory_finished_ = true;
  }

protected:

  /** @brief Pose reference */
  Eigen::Affine3d pose_reference_;
  /** @brief Twist reference */
  Eigen::Vector6d twist_reference_;
  /** @brief Initial pose for the trajectory generation */
  Eigen::Affine3d initial_pose_;
  /** @brief Internal pose of the trajectory */
  Eigen::Affine3d pose_;
  /** @brief Internal twist of the trajectory */
  Eigen::Vector6d twist_;
  double time_;
  std::atomic<double> swing_frequency_;
  std::atomic<double> length_;
  std::atomic<double> heading_; // This includes the heading w.r.t the horizontal frame and world
  std::atomic<double> heading_rate_;
  std::atomic<double> height_;
  bool trajectory_finished_;

  virtual const Eigen::Affine3d& trajectoryFunction(const double& time) = 0;
  virtual const Eigen::Vector6d& trajectoryFunctionDot(const double& time) = 0;
};

class Ellipse : public TrajectoryInterface
{

public:

  Ellipse()
  {
    xyz = xyz_rotated = xyz_dot = Eigen::Vector3d::Zero();
    world_Rz_swing = Sz = Ear = Eigen::Matrix3d::Zero();
  }

protected:

  const Eigen::Affine3d& trajectoryFunction(const double& time)
  {
    const double& yaw = heading_;

    xyz(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
    xyz(1) = 0.0;
    xyz(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

    double c = std::cos(yaw);
    double s = std::sin(yaw);

    world_Rz_swing(0,0) = c;
    world_Rz_swing(0,1) = -s;
    world_Rz_swing(1,0) = s;
    world_Rz_swing(1,1) = c;
    world_Rz_swing(2,2) = 1;

    xyz_rotated = world_Rz_swing * xyz;

    pose_.translation() = initial_pose_.translation() + xyz_rotated;

    return pose_;
  }

  const Eigen::Vector6d& trajectoryFunctionDot(const double& time)
  {
    const double& yaw_rate = heading_rate_;
    const double& yaw = heading_;

    rpy_rates.setZero();
    omegas.setZero();
    rpy.setZero();

    rpy_rates(2) = yaw_rate;
    rpy(2) = yaw;

    rpyToEar(rpy,Ear);

    omegas = Ear * rpy_rates;

    Sz(0,1) = -omegas(2);
    Sz(1,0) = -omegas(2);

    xyz_dot(0) = M_PI * swing_frequency_ * length_/2 * std::sin(M_PI * (swing_frequency_ * time));
    xyz_dot(1) = 0.0;
    xyz_dot(2) = M_PI * swing_frequency_ * height_ * std::cos(M_PI * (swing_frequency_ * time));

    twist_.setZero(); // No angular velocities
    twist_.head(3) = Sz * xyz_rotated + world_Rz_swing * xyz_dot;

    return twist_;
  }

private:

  Eigen::Vector3d xyz;
  Eigen::Vector3d xyz_dot;
  Eigen::Vector3d xyz_rotated;
  Eigen::Matrix3d world_Rz_swing;
  Eigen::Matrix3d Sz;
  Eigen::Matrix3d Ear;

  Eigen::Vector3d rpy;
  Eigen::Vector3d rpy_rates;
  Eigen::Vector3d omegas;
};

} // namespace

#endif
