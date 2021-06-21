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

QuadrupedRobot::QuadrupedRobot(ros::NodeHandle& root_nh)
{

  // Create the ModelInterface from XBot
  XBot::ConfigOptions opt;
  std::string urdf, srdf;

  nh_ = root_nh;

  if(!nh_.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
  {
      throw std::runtime_error("No robot_description given in namespace /");
  }
  if(!nh_.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
  {
      throw std::runtime_error("No robot_semantic_description given in namespace /");
  }
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

  int n_arms = xbot_model_->arms();
  int n_legs = xbot_model_->legs();
  std::vector<int> actuated_joints = xbot_model_->getEnabledJointId();

  // Load the joint names
  for(unsigned int i=0;i<actuated_joints.size();i++)
  {
      if(actuated_joints[i]>0) // Filter out the floating base joints
        joint_names_.push_back(xbot_model_->getJointByID(actuated_joints[i])->getJointName());
  }

  if(n_legs != N_LEGS)
  {
    throw std::runtime_error("Wrong number of legs, check the SRDF file!");
  }
  if(n_arms != N_ARMS)
  {
    throw std::runtime_error("Wrong number of arms, check the SRDF file!");
  }
  arm_name_ = xbot_model_->arm(0).getTipLinkName();

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

  // Check if the joint names in the ROS config file are in the same order as the one in the virtual model:
  //assert(_dof_names.size() == joint_names_.size()+FLOATING_BASE_DOFS);
  //for(unsigned int i=0;i<joint_names_.size();i++)
  //    if(_dof_names[i+FLOATING_BASE_DOFS]!=joint_names_[i])
  //    {
  //        throw std::runtime_error("Joint names in the robot model type "<<model_type<< " are not in the same order as in the ROS config file of the controller.");
  //        return false;
  //    }


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

const std::string& QuadrupedRobot::getArmName() const
{
  return arm_name_;
}

XBot::ModelInterface::Ptr QuadrupedRobot::getXBotModel()
{
  return xbot_model_;
}

const ros::NodeHandle& QuadrupedRobot::getRosNode() const
{
  return nh_;
}

ros::NodeHandle& QuadrupedRobot::getRosNode()
{
  return nh_;
}







};
