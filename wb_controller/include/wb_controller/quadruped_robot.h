#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

// ADVR
#include <XBotCoreModel/XBotCoreModel.h>
#include <ModelInterfaceRBDL/ModelInterfaceRBDL.h>
// STD
#include <memory>

namespace wb_controller
{

class QuadrupedRobot : public XBot::ModelInterfaceRBDL
{

public:

  const std::string CLASS_NAME = "QuadrupedRobot";

  typedef std::shared_ptr<QuadrupedRobot> Ptr;

  typedef std::shared_ptr<const QuadrupedRobot> ConstPtr;

  typedef std::map<std::string,std::vector<std::string> > limb_joint_names_map_t;
  typedef std::map<std::string,std::vector<int> >         limb_joint_idxs_map_t;
  typedef std::map<std::string,int >                      joint_idxs_map_t;

  enum robot_states_t {INIT,WALKING,MANIPULATION};

  QuadrupedRobot(const std::string& urdf, const std::string& srdf);

  const std::vector<std::string>& getFootNames() const;
  const std::vector<std::string>& getLegNames() const;
  const std::vector<std::string>& getHipNames() const;
  const std::vector<std::string>& getJointNames() const;
  const std::vector<std::string>& getArmEndEffectorNames() const;
  const std::vector<std::string>& getContactNames() const;
  const std::vector<std::string>& getLimbNames() const;

  const std::vector<int>& getLegJointsIds(const std::string& leg_name);
  const std::vector<int>& getArmJointsIds(const std::string& arm_name);

  const unsigned int& getNumberArms() const;
  const unsigned int& getNumberLegs() const;

  const double& getBaseLength() const;
  const double& getBaseWidth() const;

  robot_states_t getState();
  bool setState(robot_states_t robot_state);

  const Eigen::Matrix3d& getFloatingBaseInertia();

  using ModelInterface::getPose;
  using ModelInterface::getCOM;
  using ModelInterface::getCOMVelocity;
  using ModelInterface::getJacobian;

private:

  std::vector<std::string> foot_names_; // foot tip names
  std::vector<std::string> hip_names_;
  std::vector<std::string> leg_names_;
  std::vector<std::string> arm_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> ee_names_; // end-effector names
  std::vector<std::string> contact_names_; // foot + arm names
  std::vector<std::string> limb_names_; // chain names

  limb_joint_idxs_map_t joint_legs_idx_;
  limb_joint_idxs_map_t joint_arms_idx_;

  joint_idxs_map_t joint_idx_;

  limb_joint_names_map_t joint_legs_;
  limb_joint_names_map_t joint_arms_;

  unsigned int n_legs_;
  unsigned int n_arms_;

  double base_length_;
  double base_width_;

  Eigen::MatrixXd M_;
  Eigen::Matrix3d Ifb_;

  std::atomic<robot_states_t> robot_state_;

};

} //@namespace wb_controller

#endif //QUADRUPED_ROBOT_H
