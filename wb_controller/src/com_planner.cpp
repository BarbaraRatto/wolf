#include <wb_controller/com_planner.h>

using namespace rt_logger;

namespace wb_controller {

#define CLASS_NAME "ComPlanner"

ComPlanner::ComPlanner(StateEstimator::Ptr state_estimator, FootholdsPlanner::Ptr foothold_planner)
{

  state_estimator_ = state_estimator;
  foothold_planner_ = foothold_planner;

  feet_order_ = {"rf_foot","rh_foot","lh_foot","lf_foot","rf_foot"};
  com_velocity_ref_.setZero();
  com_position_ref_.setZero();

  c_ = c_ref_ = c_filt_ = 1.0;

  RtLogger::getLogger().addPublisher(CLASS_NAME"/c",c_);
  RtLogger::getLogger().addPublisher(CLASS_NAME"/com_velocity_ref",com_velocity_ref_);
}

void ComPlanner::update(double /*dt*/)
{

  // Euristic computation of a com velocity based on the
  // base linear velocity. The velocity is scaled with a value c
  // that goes from 1 if the com coincides with the base position
  // to 0 if it is close to the boundary defined by the xy position of the feet.
  // The boundary is defined by the following line segments: (rf,rh) (rh,lh) (lh,lf) (lf,rf);
  // lf ---- rf
  //  |      |
  //  |      |
  //  lh --- rh
  auto feet_pos = state_estimator_->getFeetPositionInBase();

  auto world_T_base = state_estimator_->getFloatingBasePose();

  base_X_com_ = world_T_base.inverse() * state_estimator_->getComPosition();
  p0_ = base_X_com_.head(2); // This should be defined w.r.t base

  foothold_planner_->setComCorrection(p0_);

  base_velocity_xy_ = foothold_planner_->getBaseLinearVelocityReference().head(2);
  base_velocity_xy_versor_ = base_velocity_xy_ / base_velocity_xy_.norm();

  if(base_velocity_xy_.norm() > 0)
  {

    double den = 0;
    double alpha = 0;

    for(unsigned int i=0;i<4;i++)
    {

       p1_ = feet_pos[feet_order_[i]].head(2);
       p2_ = feet_pos[feet_order_[i+1]].head(2);

       den = (base_velocity_xy_versor_.x()*(p2_.y() - p1_.y()) + base_velocity_xy_versor_.y()*(p1_.x() - p2_.x()));

       alpha =  ( p0_.x() * (p1_.y() - p2_.y()) + p0_.y() * (p2_.x() - p1_.x()) + p1_.x()*p2_.y() - p1_.y()*p2_.x() ) / den;

       p_int_.x() = p0_.x() + base_velocity_xy_versor_.x() * alpha;
       p_int_.y() = p0_.y() + base_velocity_xy_versor_.y() * alpha;

       p_int_versor_ = p_int_ / p_int_.norm();

       if(p_int_.x() > std::max(p1_.x(),p2_.x()) || p_int_.x() < std::min(p1_.x(),p2_.x()) ||
          p_int_.y() > std::max(p1_.y(),p2_.y()) || p_int_.y() < std::min(p1_.y(),p2_.y()) ||
          base_velocity_xy_versor_.head(2).dot(p_int_versor_) <= 0)
       {
         // Nothing to do
       }
       else
       {
           c_ref_ = 1.0 - std::abs((p0_.dot(p_int_versor_)) / p_int_.norm());
       }
    }
  }
  else {
    c_ref_ = 1.0; // No base velocity given
  }

  // Filter c
  c_ = secondOrderFilter(c_,c_filt_,c_ref_,0.5);

  com_velocity_ref_.setZero();
  com_velocity_ref_.head(2) = c_ * base_velocity_xy_;
  if(foothold_planner_->getGaitType() == Gait::CRAWL) // With the crawl the robot moves approx half the speed
    com_velocity_ref_ = 0.5 * com_velocity_ref_;

  //com_position_ref_(2) = foothold_planner_->getBaseHeight();
  foothold_planner_->setComVelocityRef(com_velocity_ref_);
}

const Eigen::Vector3d &ComPlanner::getComVelocity() const
{
  return com_velocity_ref_;
}

const Eigen::Vector3d &ComPlanner::getComPosition() const
{
  return com_position_ref_;
}

}; // namespace
