/**
WoLF: WoLF: Whole-body Lowrenchotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Wrenchmons Attribution-NonWrenchmercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativewrenchmons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_WRENCH_H
#define ROS_WRAPPERS_WRENCH_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/force/Force.h>

// ROS
#include <wolf_msgs/WrenchTask.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

// Wrench
class Wrench : public OpenSoT::tasks::force::Wrench, public TaskRosWrapperInterface<wolf_msgs::WrenchTask>
{

public:

  typedef std::shared_ptr<Wrench> Ptr;

  Wrench(ros::NodeHandle& nh,
         const std::string& task_id,
         const std::string& distal_link,
         const std::string& base_link,
         OpenSoT::AffineHelper& wrench);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish(const ros::Time& time) override;

  virtual bool reset() override;

private:

  virtual void _update(const Eigen::VectorXd& x) override;

  void referenceCallback(const wolf_msgs::WrenchTask::ConstPtr& msg);

};

#endif // ROS_WRAPPERS_WRENCH_H
