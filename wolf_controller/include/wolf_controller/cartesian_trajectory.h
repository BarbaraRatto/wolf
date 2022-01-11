#ifndef CARTESIAN_TRAJECTORY_H
#define CARTESIAN_TRAJECTORY_H

#include <memory>
#include <Eigen/Core>
#include <cartesian_interface/trajectory/Trajectory.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>

namespace wolf_controller
{

class CartesianTrajectory
{

public:

  enum state_t {ONLINE=0,REACHING};
  enum type_t {INTERACTIVE=0,BATCH};

  const std::string CLASS_NAME = "CartesianTrajectory";

  /**
   * @brief Shared pointer to CartesianTrajectory
   */
  typedef std::shared_ptr<CartesianTrajectory> Ptr;

  /**
   * @brief Shared pointer to const CartesianTrajectory
   */
  typedef std::shared_ptr<const CartesianTrajectory> ConstPtr;

  CartesianTrajectory(OpenSoT::tasks::acceleration::Cartesian* const task_ptr = nullptr);

  void update(double time, double period);

  void reset();

  bool getReference(Eigen::Affine3d& T_ref,
                    Eigen::Vector6d* vel_ref = nullptr,
                    Eigen::Vector6d* acc_ref = nullptr) const;

  bool setWayPoint(const Eigen::Affine3d& T_ref, double time);

private:

  double time_;

  state_t state_;
  type_t type_;

  Eigen::Affine3d T_;
  Eigen::Vector6d vel_;
  Eigen::Vector6d acc_;

  XBot::Cartesian::Trajectory::Ptr trajectory_;
  OpenSoT::tasks::acceleration::Cartesian* task_ptr_;

  bool check_reach() const;
};


} // namespace

#endif
