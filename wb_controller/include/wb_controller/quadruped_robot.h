#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

// ROS
#include <ros/ros.h>
// ADVR
#include <XBotCoreModel/XBotCoreModel.h>
#include <XBotInterface/ModelInterface.h>
// STD
#include <memory>

namespace wb_controller
{

class QuadrupedRobot
{

public:

  typedef std::shared_ptr<QuadrupedRobot> Ptr;

  typedef std::shared_ptr<const QuadrupedRobot> ConstPtr;

  QuadrupedRobot(ros::NodeHandle& root_nh);

  const std::vector<std::string>& getFootNames() const;
  const std::vector<std::string>& getHipNames() const;
  const std::vector<std::string>& getJointNames() const;
  const std::string& getArmName() const;
  XBot::ModelInterface::Ptr getXBotModel();
  const ros::NodeHandle& getRosNode() const;
  ros::NodeHandle& getRosNode();

private:
  XBot::ModelInterface::Ptr xbot_model_;
  std::vector<std::string> foot_names_;
  std::vector<std::string> hip_names_;
  std::vector<std::string> joint_names_;
  std::string arm_name_;
  ros::NodeHandle nh_;



};

} //@namespace wb_controller

#endif //QUADRUPED_ROBOT_H
