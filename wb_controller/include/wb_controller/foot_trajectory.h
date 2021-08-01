#ifndef FOOT_TRAJECTORY_H
#define FOOT_TRAJECTORY_H

#include <atomic>
#include <Eigen/Dense>

namespace wb_controller
{

class TrajectoryInterface
{

public:

  const std::string CLASS_NAME = "TrajectoryInterface";

  TrajectoryInterface()
  {
    pose_reference_ = initial_pose_ = pose_ = Eigen::Affine3d::Identity();
    twist_reference_.setZero();
    twist_.setZero();
    swing_frequency_ = 0.0;
    time_ = 0.0;
    length_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
    yaw_rate_ = 0.0;
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

  void setStepYawRate(const double& yaw_rate)
  {
    yaw_rate_ = yaw_rate;
  }

  void setStepRollRate(const double& roll_rate)
  {
    roll_rate_ = roll_rate;
  }

  void setStepPitchRate(const double& pitch_rate)
  {
    pitch_rate_ = pitch_rate;
  }

  void setStepLength(const double& length)
  {
    length_ = length;
  }

  void setStepRoll(const double& roll)
  {
    roll_ = roll;
  }

  void setStepPitch(const double& pitch)
  {
    pitch_ = pitch;
  }

  void setStepYaw(const double& yaw)
  {
    yaw_ = yaw;
  }

  void setStepHeight(const double& height)
  {
    height_ = height;
  }

  double getStepLength()
  {
    return length_;
  }

  double getStepRoll()
  {
    return roll_;
  }

  double getStepPitch()
  {
    return pitch_;
  }

  double getStepYaw()
  {
    return yaw_;
  }

  double getStepYawRate()
  {
    return yaw_rate_;
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

  /** @brief This is the roll w.r.t the world frame estimated through the terrain estimator */
  std::atomic<double> roll_;
  /** @brief This is the pitch w.r.t the world frame estimated through the terrain estimator */
  std::atomic<double> pitch_;
  /** @brief This is the yaw w.r.t the horizontal frame and world */
  std::atomic<double> yaw_;

  std::atomic<double> roll_rate_;
  std::atomic<double> pitch_rate_;
  std::atomic<double> yaw_rate_;
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
    world_R_swing = swing_R_world = S = Ear = Eigen::Matrix3d::Zero();
  }

protected:

  const Eigen::Affine3d& trajectoryFunction(const double& time)
  {
    rpy(0) = roll_;
    rpy(1) = pitch_;
    rpy(2) = yaw_;

    xyz(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
    xyz(1) = 0.0;
    xyz(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

    rpyToRot(rpy,swing_R_world);

    world_R_swing.noalias() = swing_R_world.transpose();

    xyz_rotated.noalias() = world_R_swing * xyz;

    pose_.translation() = initial_pose_.translation() + xyz_rotated;

    return pose_;
  }

  const Eigen::Vector6d& trajectoryFunctionDot(const double& time)
  {

    // FIXME Where are the RATES?
    rpy_rates.setZero();
    omegas.setZero();
    rpy.setZero();

    rpy_rates(0) = roll_rate_;
    rpy_rates(1) = pitch_rate_;
    rpy_rates(2) = yaw_rate_;

    rpy(0) = roll_;
    rpy(1) = pitch_;
    rpy(2) = yaw_;

    rpyToEar(rpy,Ear);

    omegas = Ear * rpy_rates;

    S(0,1) = -omegas(2);
    S(1,0) =  omegas(2);

    S(0,2) =  omegas(1);
    S(2,0) = -omegas(1);

    S(1,2) = -omegas(0);
    S(2,1) =  omegas(0);

    xyz_dot(0) = M_PI * swing_frequency_ * length_/2 * std::sin(M_PI * (swing_frequency_ * time));
    xyz_dot(1) = 0.0;
    xyz_dot(2) = M_PI * swing_frequency_ * height_ * std::cos(M_PI * (swing_frequency_ * time));

    twist_.setZero(); // No angular velocities
    twist_.head(3) = S * xyz_rotated + world_R_swing * xyz_dot;

    return twist_;
  }

private:

  Eigen::Vector3d xyz;
  Eigen::Vector3d xyz_dot;
  Eigen::Vector3d xyz_rotated;
  Eigen::Matrix3d world_R_swing;
  Eigen::Matrix3d swing_R_world;
  Eigen::Matrix3d S;
  Eigen::Matrix3d Ear;

  Eigen::Vector3d rpy;
  Eigen::Vector3d rpy_rates;
  Eigen::Vector3d omegas;
};

} // namespace

#endif
