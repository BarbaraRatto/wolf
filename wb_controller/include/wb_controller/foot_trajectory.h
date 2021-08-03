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
    :trajectory_id(_id++)
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

    rt_logger::RtLogger::getLogger().addPublisher(CLASS_NAME+"/position_reference_"+_legs_prefix[trajectory_id],position_reference_);
    rt_logger::RtLogger::getLogger().addPublisher(CLASS_NAME+"/velocity_reference_"+_legs_prefix[trajectory_id],velocity_reference_);
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

    position_reference_ = pose_reference_.translation();
    velocity_reference_ = twist_reference_.head(3);

    if(swing_frequency_*time_>=1.0)
      trajectory_finished_ = true;
  }

protected:

  inline static int _id = 0;
  int trajectory_id;

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

  Eigen::Vector3d position_reference_;
  Eigen::Vector3d velocity_reference_;

  virtual const Eigen::Affine3d& trajectoryFunction(const double& time) = 0;
  virtual const Eigen::Vector6d& trajectoryFunctionDot(const double& time) = 0;
};

class Ellipse : public TrajectoryInterface
{

public:

  const std::string CLASS_NAME = "Ellipse";

  Ellipse()
  {
    xyz_ = xyz_rotated_ = xyz_dot_ = Eigen::Vector3d::Zero();
    terrain_R_swing_ = world_R_terrain_ = terrain_R_world_ = world_Rz_swing_ = Sz_ = Ear_ = Eigen::Matrix3d::Zero();
  }

protected:

  const Eigen::Affine3d& trajectoryFunction(const double& time)
  {
    xyz_(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
    xyz_(1) = 0.0;
    xyz_(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

    double c = std::cos(yaw_);
    double s = std::sin(yaw_);

    world_Rz_swing_(0,0) = c;
    world_Rz_swing_(0,1) = -s;
    world_Rz_swing_(1,0) = s;
    world_Rz_swing_(1,1) = c;
    world_Rz_swing_(2,2) = 1;

    rpyToRot(roll_,pitch_,0.0,world_R_terrain_);

    terrain_R_world_.noalias() = world_R_terrain_.transpose();

    terrain_R_swing_ = terrain_R_world_ * world_Rz_swing_;

    xyz_rotated_.noalias() = terrain_R_swing_ * xyz_;

    pose_.translation() = initial_pose_.translation() + xyz_rotated_;

    return pose_;
  }

  const Eigen::Vector6d& trajectoryFunctionDot(const double& time)
  {

    xyz_dot_(0) = M_PI * swing_frequency_ * length_/2 * std::sin(M_PI * (swing_frequency_ * time));
    xyz_dot_(1) = 0.0;
    xyz_dot_(2) = M_PI * swing_frequency_ * height_ * std::cos(M_PI * (swing_frequency_ * time));

    rpy_rates_.setZero();
    omegas_.setZero();
    rpy_.setZero();

    rpy_(2) = yaw_;
    rpy_rates_(2) = yaw_rate_;

    rpyToEarWorld(rpy_,Ear_);

    omegas_ = Ear_ * rpy_rates_;

    Sz_(0,1) = -omegas_(2);
    Sz_(1,0) = -omegas_(2);

    twist_.setZero(); // No angular velocities
    // We are assuming that the terrain frame does not change so fast i.e. it does not contribute to the velocities, it just rotates them
    twist_.head(3) = terrain_R_world_ * (Sz_ * world_Rz_swing_ * xyz_ + world_Rz_swing_ * xyz_dot_);

    return twist_;
  }

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
  Eigen::Matrix3d world_R_terrain_;
  Eigen::Matrix3d terrain_R_swing_;
};

} // namespace

#endif
