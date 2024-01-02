/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_MOMENTUM_H
#define ROS_WRAPPERS_MOMENTUM_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>

// ROS
#include <wolf_msgs/CartesianTask.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>
#include <wolf_controller/ros_wrappers/cartesian.h>

// AngularMomentum
class AngularMomentum : public OpenSoT::tasks::acceleration::AngularMomentum, public TaskRosWrapperInterface<wolf_msgs::CartesianTask>
{

public:

  typedef std::shared_ptr<AngularMomentum> Ptr;

  AngularMomentum(ros::NodeHandle& nh, XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void update(const Eigen::VectorXd& x);

  virtual void publish(const ros::Time& time, const ros::Duration& period);


};

#endif // ROS_WRAPPERS_MOMENTUM_H

