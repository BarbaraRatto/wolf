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
  :ModelInterfaceRBDL(), robot_state_(robot_states_t::INIT)
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
  //model_imp_ = XBot::ModelInterface::getModel(opt);

  _is_floating_base = true;
  init(opt);
  //init_model(opt);

  _dof_names = getEnabledJointNames();

  for(unsigned int i=0;i<_dof_names.size();i++)
  {
    joint_idx_[_dof_names[i]] = i;
    if(i>5) // Remove the floating base
    {
      joint_names_.push_back(_dof_names[i]);  // Load the joint names for ROS-Control
      ROS_INFO_STREAM_NAMED(CLASS_NAME,"ROS-Control joints order: " << joint_names_[joint_names_.size()-1]);
    }
  }

  const srdf_advr::Model& srdf_model = getSrdf();

  for(unsigned int i=0;i < srdf_model.getGroups().size(); i++)
  {
    const auto& chains = srdf_model.getGroups()[i].chains_;
    const auto& links = srdf_model.getGroups()[i].links_;
    const auto& joints = srdf_model.getGroups()[i].joints_;
    // Parse the foot tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("leg") != std::string::npos)
    {
      leg_names_.push_back(srdf_model.getGroups()[i].name_);
      for(unsigned int j=0;j<chains.size();j++)
        foot_names_.push_back(chains[j].second);
      for(unsigned int j=0;j<joints.size();j++)
        joint_legs_[srdf_model.getGroups()[i].name_].push_back(joints[j]);
    }
    // Parse the arm tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("arm") != std::string::npos)
    {
      arm_names_.push_back(srdf_model.getGroups()[i].name_);
      for(unsigned int j=0;j<chains.size();j++)
        ee_names_.push_back(chains[j].second);
      for(unsigned int j=0;j<joints.size();j++)
        joint_arms_[srdf_model.getGroups()[i].name_].push_back(joints[j]);
    }
    // Parse the hip tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("hip") != std::string::npos)
    {
      for(unsigned int j=0;j<chains.size();j++)
        hip_names_.push_back(chains[j].second);
    }
    // Parse the base link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("bases") != std::string::npos)
    {
      for(unsigned int j=0;j<links.size();j++)
        base_names_.push_back(links[j]);
    }
  }

  n_legs_ = leg_names_.size();
  n_arms_ = arm_names_.size();

  // Check the numbers
  if(n_legs_ != N_LEGS)
  {
    throw std::runtime_error("Wrong number of legs, check the SRDF file!");
  }
  if(n_arms_ > N_ARMS)
  {
    throw std::runtime_error("Wrong number of arms, check the SRDF file!");
  }
  if(foot_names_.size() != N_LEGS)
  {
    throw std::runtime_error("Wrong number of feet, check the SRDF file!");
  }
  if(hip_names_.size() != N_LEGS)
  {
    throw std::runtime_error("Wrong number of hips, check the SRDF file!");
  }
  if(base_names_.size() > N_BASES)
  {
    throw std::runtime_error("Wrong number of bases, check the SRDF file!");
  }

  for(unsigned int i=0;i<leg_names_.size();i++)
  {
    for(unsigned int j=0;j<joint_legs_[leg_names_[i]].size();j++)
    {
      std::string current_joint_name = joint_legs_[leg_names_[i]].at(j);
      int idx = joint_idx_[current_joint_name];
      joint_legs_idx_[leg_names_[i]].push_back(idx);
      ROS_INFO_STREAM_NAMED(CLASS_NAME,leg_names_[i] << " " << joint_legs_[leg_names_[i]][j] << " " << idx);

    }
  }

  for(unsigned int i=0;i<arm_names_.size();i++)
  {
    for(unsigned int j=0;j<joint_arms_[arm_names_[i]].size();j++)
    {
      std::string current_joint_name = joint_arms_[arm_names_[i]].at(j);
      int idx = joint_idx_[current_joint_name];
      joint_arms_idx_[arm_names_[i]].push_back(idx);
      ROS_INFO_STREAM_NAMED(CLASS_NAME,arm_names_[i] << " " << joint_arms_[arm_names_[i]][j] << " " << idx);
    }
  }

  hip_names_ = sortByLegPrefix(hip_names_);
  foot_names_ = sortByLegPrefix(foot_names_);
  leg_names_ = sortByLegPrefix(leg_names_);

  std::vector<std::string> limbs;
  limbs = getChainNames();

  for(unsigned int i = 0; i < limbs.size(); i++) // Remove virtual_chain
      if(limbs[i].find("virtual_chain") == std::string::npos)
          limb_names_.push_back(limbs[i]);

  contact_names_ = foot_names_;
  contact_names_.insert( contact_names_.end(), ee_names_.begin(), ee_names_.end() );

  // Calculate approx base length and width based on the hip positions
  // Hips order: "lf","lh","rf","rh"
  Eigen::Affine3d pose_lf, pose_lh, pose_rf, pose_rh;
  ModelInterface::getPose(hip_names_[0],BASE_LINK_FRAME_NAME,pose_lf);
  ModelInterface::getPose(hip_names_[1],BASE_LINK_FRAME_NAME,pose_lh);
  ModelInterface::getPose(hip_names_[2],BASE_LINK_FRAME_NAME,pose_rf);
  ModelInterface::getPose(hip_names_[3],BASE_LINK_FRAME_NAME,pose_rh);
  base_width_  = std::abs(pose_lf.translation().y() - pose_rf.translation().y());
  base_length_ = std::abs(pose_lf.translation().x() - pose_lh.translation().x());
}

const std::vector<std::string>& QuadrupedRobot::getFootNames() const
{
  return foot_names_;
}

const std::vector<std::string> &QuadrupedRobot::getLegNames() const
{
  return leg_names_;
}

const std::vector<std::string>& QuadrupedRobot::getHipNames() const
{
  return hip_names_;
}

const std::vector<std::string>& QuadrupedRobot::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string>& QuadrupedRobot::getArmEndEffectorNames() const
{
  return ee_names_;
}

const std::vector<std::string>& QuadrupedRobot::getContactNames() const
{
  return contact_names_;
}

const std::vector<std::string>& QuadrupedRobot::getLimbNames() const
{
  return limb_names_;
}

const std::vector<int> &QuadrupedRobot::getLegJointsIds(const std::string &leg_name)
{
  return joint_legs_idx_[leg_name];
}

const std::vector<int> &QuadrupedRobot::getArmJointsIds(const std::string &arm_name)
{
  return joint_arms_idx_[arm_name];
}

const unsigned int& QuadrupedRobot::getNumberArms() const
{
  return n_arms_;
}

const unsigned int& QuadrupedRobot::getNumberLegs() const
{
  return n_legs_;
}

const double &QuadrupedRobot::getBaseLength() const
{
  return base_length_;
}

const double &QuadrupedRobot::getBaseWidth() const
{
  return base_width_;
}

std::string QuadrupedRobot::getTrunkLinkName()
{
  return base_names_[0];
}

std::string QuadrupedRobot::getArmBaseLinkName()
{
  if(base_names_.size()>1)
    return base_names_[1];
  else
    return "";
}

const Eigen::Matrix3d& QuadrupedRobot::getFloatingBaseInertia()
{
  getInertiaMatrix(M_);
  Ifb_ = M_.block(3,3,3,3);
  return Ifb_;
}

QuadrupedRobot::robot_states_t QuadrupedRobot::getState()
{
  return robot_state_;
}

bool QuadrupedRobot::setState(QuadrupedRobot::robot_states_t state)
{
  robot_state_ = state;
  return true;
}


};
