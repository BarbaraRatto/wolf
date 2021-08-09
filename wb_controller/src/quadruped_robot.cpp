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
  :ModelInterfaceRBDL()

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

  init(opt);
  init_model(opt);

  _dof_names = getEnabledJointNames();

  n_arms_ = arms();
  n_legs_ = legs();
  std::vector<int> actuated_joints = getEnabledJointId();

  // Load the joint names
  for(unsigned int i=0;i<actuated_joints.size();i++)
  {
      if(actuated_joints[i]>0) // Filter out the floating base joints
        joint_names_.push_back(getJointByID(actuated_joints[i])->getJointName());
  }

  if(n_legs_ != N_LEGS)
  {
    throw std::runtime_error("Wrong number of legs, check the SRDF file!");
  }
  if(n_arms_ > N_ARMS)
  {
    throw std::runtime_error("Wrong number of arms, check the SRDF file!");
  } 

  const srdf_advr::Model& srdf_model = getSrdf();
  for(unsigned int i=0;i < srdf_model.getGroups().size(); i++)
  {
    const auto& chains = srdf_model.getGroups()[i].chains_;
    const auto& links = srdf_model.getGroups()[i].links_;
    // Parse the foot tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("leg") != std::string::npos)
    {
      for(unsigned int j=0;j<chains.size();j++)
        foot_names_.push_back(chains[j].second);
    }
    // Parse the arm tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("arm") != std::string::npos)
    {
      for(unsigned int j=0;j<chains.size();j++)
        arm_names_.push_back(chains[j].second);
    }
    // Parse the hip tip_link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("hip") != std::string::npos)
    {
      for(unsigned int j=0;j<chains.size();j++)
        hip_names_.push_back(chains[j].second);
    }
    // Parse the base link from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("base") != std::string::npos)
    {
      for(unsigned int j=0;j<links.size();j++)
        base_names_.push_back(links[j]);
    }
  }

  // Check the numbers
  if(foot_names_.size() != N_LEGS)
  {
    throw std::runtime_error("Wrong number of feet, check the SRDF file!");
  }
  if(hip_names_.size() != N_LEGS)
  {
    throw std::runtime_error("Wrong number of hips, check the SRDF file!");
  }
  if(base_names_.size() != N_BASES)
  {
    throw std::runtime_error("Wrong number of bases, check the SRDF file!");
  }

  hip_names_ = sortByLegPrefix(hip_names_);
  foot_names_ = sortByLegPrefix(foot_names_);

  std::vector<std::string> limbs;
  limbs = getChainNames();

  for(unsigned int i = 0; i < limbs.size(); i++) // Remove virtual_chain
      if(limbs[i].find("virtual_chain") == std::string::npos)
          limb_names_.push_back(limbs[i]);

  contact_names_ = foot_names_;
  contact_names_.insert( contact_names_.end(), arm_names_.begin(), arm_names_.end() );

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

const std::string &QuadrupedRobot::getBaseLinkName() const
{
  return base_names_[0];
}

const Eigen::Matrix3d &QuadrupedRobot::getFloatingBaseInertia()
{
  getInertiaMatrix(M_);
  Ifb_ = M_.block(3,3,3,3);
  return Ifb_;
}

QuadrupedRobot::states_t QuadrupedRobot::getState()
{
  return state_;
}

bool QuadrupedRobot::setState(QuadrupedRobot::states_t state)
{
  state_ = state;
  return true;
}


};
