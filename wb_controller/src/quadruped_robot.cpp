/**
 * @file robot_model.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief WholeBody Controller.
 *
 * @see todo.git
 */

#include <wb_controller/quadruped_robot.h>
#include <wb_controller/utils.h>
#include <stdexcept>

using namespace XBot;

namespace wb_controller {

QuadrupedRobot::QuadrupedRobot(const std::string& urdf, const std::string& srdf)
{

  // Create the ModelInterface from XBot
  XBot::ConfigOptions opt;

  if(!opt.set_urdf(urdf))
  {
      throw std::runtime_error("Unable to load URDF file");
  }
  if(!opt.set_srdf(srdf))
  {
      throw std::runtime_error("Unable to load SRDF file");
  }
  if(!opt.generate_jidmap())
  {
      throw std::runtime_error("Unable to load jidmap");
  }

  opt.set_parameter("is_model_floating_base", true);
  std::string model_type = "RBDL";
  opt.set_parameter<std::string>("model_type", model_type);
  xbot_model_ = XBot::ModelInterface::getModel(opt);

  _dof_names = xbot_model_->getEnabledJointNames();

  n_arms_ = xbot_model_->arms();
  n_legs_ = xbot_model_->legs();
  std::vector<int> actuated_joints = xbot_model_->getEnabledJointId();

  // Load the joint names
  for(unsigned int i=0;i<actuated_joints.size();i++)
  {
      if(actuated_joints[i]>0) // Filter out the floating base joints
        joint_names_.push_back(xbot_model_->getJointByID(actuated_joints[i])->getJointName());
  }

  if(n_legs_ != N_LEGS)
  {
    throw std::runtime_error("Wrong number of legs, check the SRDF file!");
  }
  if(n_arms_ > N_ARMS)
  {
    throw std::runtime_error("Wrong number of arms, check the SRDF file!");
  }
  if(n_arms_ == N_ARMS) // Note: in the future we could have more than 1 ARM
    for(unsigned int i = 0; i<n_arms_; i++)
      arm_names_.push_back(xbot_model_->arm(i).getTipLinkName());

  const srdf_advr::Model& srdf_model = xbot_model_->getSrdf();
  for(unsigned int i=0;i < srdf_model.getGroups().size(); i++)
  {
    const auto& chains = srdf_model.getGroups()[i].chains_;
    // Parse the foot tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("leg") != std::string::npos)
    {
      for(unsigned int j=0;j<chains.size();j++)
        foot_names_.push_back(chains[j].second);
    }
    // Parse the hip tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("hip") != std::string::npos)
    {
      for(unsigned int j=0;j<chains.size();j++)
        hip_names_.push_back(chains[j].second);
    }
  }

  // Check the number of feet and hips
  if(foot_names_.size() != N_LEGS)
  {
    throw std::runtime_error("Wrong number of feet, check the SRDF file!");
  }
  if(hip_names_.size() != N_LEGS)
  {
    throw std::runtime_error("Wrong number of hips, check the SRDF file!");
  }


  hip_names_ = sortByLegPrefix(hip_names_);
  foot_names_ = sortByLegPrefix(foot_names_);

  std::vector<std::string> limbs;
  limbs = xbot_model_->getChainNames();

  for(unsigned int i = 0; i < limbs.size(); i++) // Remove virtual_chain
      if(limbs[i].find("virtual_chain") == std::string::npos)
          limb_names_.push_back(limbs[i]);

  limb_names_ = sortByLegPrefix(limb_names_);

  contact_names_ = foot_names_;
  contact_names_.insert( contact_names_.end(), arm_names_.begin(), arm_names_.end() );

}

const std::vector<std::string>& QuadrupedRobot::getFootNames() const
{
  return foot_names_;
}

const std::vector<std::string>& QuadrupedRobot::getHipNames() const
{
  return hip_names_;
}

const std::vector<std::string>& QuadrupedRobot::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string>& QuadrupedRobot::getArmNames() const
{
  return arm_names_;
}

const std::vector<std::string>& QuadrupedRobot::getContactNames() const
{
  return contact_names_;
}

const std::vector<std::string>& QuadrupedRobot::getLimbNames() const
{
  return limb_names_;
}

XBot::ModelInterface::Ptr QuadrupedRobot::getXBotModel()
{
  return xbot_model_;
}

const unsigned int& QuadrupedRobot::getNumberArms() const
{
  return n_arms_;
}

const unsigned int& QuadrupedRobot::getNumberLegs() const
{
  return n_legs_;
}

};
