/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_COM_H
#define ROS_WRAPPERS_COM_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/CoM.h>

// ROS
#include <wolf_msgs/ComTask.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

// Com
class Com : public OpenSoT::tasks::acceleration::CoM, public TaskRosWrapperInterface<wolf_msgs::ComTask>
{

public:

  typedef std::shared_ptr<Com> Ptr;

  Com(ros::NodeHandle& nh, const XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish(const ros::Time& time, const ros::Duration& period) override;

protected:

  void referenceCallback(const wolf_msgs::ComTask::ConstPtr& msg);

private:

  virtual void _update(const Eigen::VectorXd& x) override;

};

#endif // ROS_WRAPPERS_COM_H

