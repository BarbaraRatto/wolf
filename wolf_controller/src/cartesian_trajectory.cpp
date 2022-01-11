#include <wolf_controller/cartesian_trajectory.h>

using namespace XBot;
using namespace Cartesian;

namespace
{
const double DEFAULT_REACH_THRESHOLD = 1e-6;
}

namespace wolf_controller {

CartesianTrajectory::CartesianTrajectory(OpenSoT::tasks::acceleration::Cartesian* const task_ptr)
  :type_(type_t::INTERACTIVE)
  ,task_ptr_(task_ptr)
{
  trajectory_ = std::make_shared<Trajectory>();
  time_ = 0.0;
  reset();
}

void CartesianTrajectory::reset()
{
  state_ = state_t::ONLINE;
  if(task_ptr_!=nullptr)
    task_ptr_->getActualPose(T_);
  else
    T_ = Eigen::Affine3d::Identity();
  vel_.setZero();
  acc_.setZero();
  trajectory_->clear();
}

void CartesianTrajectory::update(double time, double period)
{
    time_ = time;

    switch(state_)
    {
        case state_t::REACHING:
          T_ = trajectory_->evaluate(time, &vel_, &acc_);
          if(trajectory_->isTrajectoryEnded(time) && check_reach())
            state_ = state_t::ONLINE;
        break;

        case state_t::ONLINE:
          vel_.setZero();
          acc_.setZero();
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

bool CartesianTrajectory::setWayPoint(const Eigen::Affine3d& T_ref, double time)
{
  if(state_ == state_t::REACHING)
    return false;

  state_ = state_t::REACHING;

  if(type_ == type_t::INTERACTIVE)
  {
    trajectory_->clear();
    trajectory_->addWayPoint(time_, T_);
    trajectory_->addWayPoint(time_ + time, T_ref);
    trajectory_->compute();
  }
  return true;
}



bool CartesianTrajectory::check_reach() const
{
    auto T_ref = trajectory_->getWayPoints().back().frame;
    return T_ref.isApprox(T_, DEFAULT_REACH_THRESHOLD);
}

}; // namespace
