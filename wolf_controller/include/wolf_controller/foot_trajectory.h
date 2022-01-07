#ifndef FOOT_TRAJECTORY_H
#define FOOT_TRAJECTORY_H

#include <atomic>
#include <Eigen/Dense>

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

class Ellipse : public TrajectoryInterface
{

public:

  const std::string CLASS_NAME = "Ellipse";

  Ellipse()
  {
    xyz_ = xyz_rotated_ = xyz_dot_ = Eigen::Vector3d::Zero();
    terrain_R_swing_ = terrain_R_world_ = world_Rz_swing_ = Sz_ = Ear_ = Eigen::Matrix3d::Zero();
  }

protected:

  const Eigen::Affine3d& trajectoryFunction(const double& time)
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

  const Eigen::Vector6d& trajectoryFunctionDot(const double& time)
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
