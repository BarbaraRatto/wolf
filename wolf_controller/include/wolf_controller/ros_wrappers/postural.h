/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_POSTURAL_H
#define ROS_WRAPPERS_POSTURAL_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Postural.h>

// ROS
#include <wolf_msgs/PosturalTask.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>

// POSTURAL
class Postural : public OpenSoT::tasks::acceleration::Postural, public TaskRosWrapperInterface<wolf_msgs::PosturalTask>
{

public:

  typedef std::shared_ptr<Postural> Ptr;

  Postural(ros::NodeHandle& nh, const XBot::ModelInterface& robot,
           OpenSoT::AffineHelper qddot = OpenSoT::AffineHelper(), const std::string task_id = "postural");

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish(const ros::Time& time, const ros::Duration& period);

private:

  virtual void _update(const Eigen::VectorXd& x) override;

};

#endif // ROS_WRAPPERS_POSTURAL_H

