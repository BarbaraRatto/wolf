#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

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

  QuadrupedRobot(const std::string& urdf, const std::string& srdf);

  const std::vector<std::string>& getFootNames() const;
  const std::vector<std::string>& getHipNames() const;
  const std::vector<std::string>& getJointNames() const;
  const std::vector<std::string>& getArmNames() const;
  const std::vector<std::string>& getContactNames() const;
  const std::vector<std::string>& getLimbNames() const;
  XBot::ModelInterface::Ptr getXBotModel();

  const unsigned int& getNumberArms() const;
  const unsigned int& getNumberLegs() const;

private:

  XBot::ModelInterface::Ptr xbot_model_;
  std::vector<std::string> foot_names_; // foot tip names
  std::vector<std::string> hip_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> arm_names_; // arm tip names
  std::vector<std::string> contact_names_; // foot + arm names
  std::vector<std::string> limb_names_; // chain names

  unsigned int n_legs_;
  unsigned int n_arms_;

};

} //@namespace wb_controller

#endif //QUADRUPED_ROBOT_H
