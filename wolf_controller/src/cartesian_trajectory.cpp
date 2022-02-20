/**
 * @file com_planner.cpp
 * @author Gennaro Raiola
 * @date 1 December, 2021
 * @brief Cartesian trajectory
 */

#include <wolf_controller/cartesian_trajectory.h>

using namespace XBot;
using namespace Cartesian;

namespace
{
const double DEFAULT_REACH_THRESHOLD = 1e-6;
}

namespace wolf_controller {

CartesianTrajectory::CartesianTrajectory(OpenSoT::tasks::acceleration::Cartesian* const task_ptr)
  :state_(state_t::IDLE)
  ,task_ptr_(task_ptr)
{
  trajectory_ = std::make_shared<Trajectory>();
  reset();
}

void CartesianTrajectory::reset()
{
  state_ = state_t::IDLE;
  if(task_ptr_!=nullptr)
    task_ptr_->getActualPose(T_);
  else
    T_ = Eigen::Affine3d::Identity();
  vel_.setZero();
  acc_.setZero();
  trajectory_->clear();
  time_ = 0.0;
}

void CartesianTrajectory::update(double period)
{
  switch(state_)
  {
  case state_t::EXECUTING:
    T_ = trajectory_->evaluate(time_, &vel_, &acc_);
    if(trajectory_->isTrajectoryEnded(time_) && checkProximity())
      state_ = state_t::IDLE;
    time_ = time_ + period;
    break;

  case state_t::IDLE:
    vel_.setZero();
    acc_.setZero();
    time_ = 0.0;
    break;
  };
}

bool CartesianTrajectory::getReference(Eigen::Affine3d& T_ref,
                                       Eigen::Vector6d* vel_ref,
                                       Eigen::Vector6d* acc_ref) const
{
  T_ref = T_;

  if(vel_ref) *vel_ref = vel_;
  if(acc_ref) *acc_ref = acc_;

  return true;
}

bool CartesianTrajectory::setWayPoint(const Eigen::Affine3d& T_ref, double duration)
{
  if(state_ == state_t::EXECUTING)
    return false;

  state_ = state_t::EXECUTING;

  trajectory_->clear();
  trajectory_->addWayPoint(0.0, T_);
  trajectory_->addWayPoint(duration, T_ref);
  trajectory_->compute();

  return true;
}

bool CartesianTrajectory::setWayPoints(const std::vector<WayPoint> &wps)
{
  if(state_ == state_t::EXECUTING)
    return false;

  state_ = state_t::EXECUTING;

  trajectory_->clear();
  trajectory_->addWayPoint(0.0, T_);
  for(unsigned int i=0; i<wps.size();i++)
    trajectory_->addWayPoint(wps[i].duration, wps[i].T_ref);
  trajectory_->compute();

  return true;
}

CartesianTrajectory::state_t CartesianTrajectory::getState() const
{
  return state_;
}

bool CartesianTrajectory::checkProximity() const
{
  auto T_ref = trajectory_->getWayPoints().back().frame;
  return T_ref.isApprox(T_, DEFAULT_REACH_THRESHOLD);
}

double wolf_controller::CartesianTrajectory::getTime() const
{
  return time_;
}

}; // namespace
