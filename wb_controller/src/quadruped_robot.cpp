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

  for(unsigned int i=0;i<leg_names_.size();i++)
  {
    for(unsigned int j=0;j<joint_legs_[leg_names_[i]].size();j++)
    {
      std::string current_joint_name = joint_legs_[leg_names_[i]].at(j);
      int idx = joint_idx_[current_joint_name];
      joint_limb_idx_[leg_names_[i]].push_back(idx);
      ROS_INFO_STREAM_NAMED(CLASS_NAME,leg_names_[i] << " " << joint_legs_[leg_names_[i]][j] << " " << idx);
    }
  }

  for(unsigned int i=0;i<arm_names_.size();i++)
  {
    for(unsigned int j=0;j<joint_arms_[arm_names_[i]].size();j++)
    {
      std::string current_joint_name = joint_arms_[arm_names_[i]].at(j);
      int idx = joint_idx_[current_joint_name];
      joint_limb_idx_[arm_names_[i]].push_back(idx);
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

  // Initializations
  world_T_base_ = Eigen::Affine3d::Identity();
  world_R_hf_ = world_R_base_ = hf_R_base_ = Eigen::Matrix3d::Identity();
  for(unsigned int i=0;i<foot_names_.size();i++)
  {
    world_X_foot_[foot_names_[i]] = Eigen::Vector3d::Zero();
    base_X_foot_[foot_names_[i]] = Eigen::Vector3d::Zero();
    world_T_foot_[foot_names_[i]] = Eigen::Affine3d::Identity();
    base_T_foot_[foot_names_[i]] = Eigen::Affine3d::Identity();
  }
  for(unsigned int i=0;i<arm_names_.size();i++)
  {
    world_X_arm_[arm_names_[i]] = Eigen::Vector3d::Zero();
    base_X_arm_[arm_names_[i]] = Eigen::Vector3d::Zero();
    world_T_arm_[arm_names_[i]] = Eigen::Affine3d::Identity();
    base_T_arm_[arm_names_[i]] = Eigen::Affine3d::Identity();
  }

  getInertiaMatrix(tmp_M_);
  getInertiaInverse(tmp_Mi_);
}

bool QuadrupedRobot::update(bool update_position, bool update_velocity, bool update_desired_acceleration)
{
  ROS_DEBUG_NAMED(CLASS_NAME,"update");

  // Internal update
  bool res = ModelInterface::update(update_position,update_velocity,update_desired_acceleration);

  // Update the transformations between world, base and horizontal frame
  getPose(BASE_LINK_FRAME_NAME,world_T_base_);
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_T_base.translation()" << world_T_base_.translation());
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_T_base.linear()" << world_T_base_.linear());
  world_R_hf_ = Eigen::Matrix3d::Identity();
  yaw_base_   = std::atan2(world_T_base_.linear()(1,0),world_T_base_.linear()(0,0));
  world_R_hf_ = Eigen::AngleAxisd(yaw_base_,Eigen::Vector3d::UnitZ());
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"yaw_base" << yaw_base_);
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_R_hf" << world_R_hf_);
  world_R_base_ = world_T_base_.linear();
  hf_R_base_ = world_R_hf_.transpose() * world_R_base_;
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_R_base" << world_R_base_);
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_R_base" << hf_R_base_);

  // Update the limb transformations
  for(unsigned int i=0; i<foot_names_.size(); i++)
  {
    // Feet position in world
    getPose(foot_names_[i],world_T_foot_[foot_names_[i]]);
    world_X_foot_[foot_names_[i]] = world_T_foot_[foot_names_[i]].translation();
    // Feet position in base/trunk
    getPose(foot_names_[i],BASE_LINK_FRAME_NAME,base_T_foot_[foot_names_[i]]);
    base_X_foot_[foot_names_[i]] = base_T_foot_[foot_names_[i]].translation();
  }

  for(unsigned int i=0; i<ee_names_.size(); i++)
  {
    // Arms position in world
    getPose(ee_names_[i],world_T_arm_[ee_names_[i]]);
    world_X_arm_[ee_names_[i]] = world_T_arm_[ee_names_[i]].translation();
    // Arms position in base/trunk
    getPose(ee_names_[i],BASE_LINK_FRAME_NAME,base_T_arm_[ee_names_[i]]);
    base_X_arm_[ee_names_[i]] = base_T_arm_[ee_names_[i]].translation();
  }

  return res;
}

const std::map<std::string,Eigen::Vector3d>& QuadrupedRobot::getArmsPositionInWorld() const
{
  return world_X_arm_;
}

Eigen::Vector3d& QuadrupedRobot::getArmPositionInWorld(const std::string& name)
{
  return world_X_arm_[name];
}

const std::map<std::string,Eigen::Vector3d>& QuadrupedRobot::getArmsPositionInBase() const
{
  return base_X_arm_;
}

Eigen::Vector3d& QuadrupedRobot::getArmPositionInBase(const std::string& name)
{
  return base_X_arm_[name];
}

const std::map<std::string,Eigen::Affine3d>& QuadrupedRobot::getArmsPoseInWorld() const
{
  return world_T_arm_;
}

const std::map<std::string,Eigen::Affine3d>& QuadrupedRobot::getArmsPoseInBase() const
{
  return base_T_arm_;
}

Eigen::Affine3d& QuadrupedRobot::getArmPoseInWorld(const std::string& name)
{
  return world_T_arm_[name];
}

Eigen::Affine3d& QuadrupedRobot::getArmPoseInBase(const std::string& name)
{
  return base_T_arm_[name];
}

const Eigen::Affine3d &QuadrupedRobot::getBasePoseInWorld() const
{
  return world_T_base_;
}

const std::map<std::string,Eigen::Vector3d>& QuadrupedRobot::getFeetPositionInWorld() const
{
  return world_X_foot_;
}

Eigen::Vector3d& QuadrupedRobot::getFootPositionInWorld(const std::string& name)
{
  return world_X_foot_[name];
}

const std::map<std::string,Eigen::Vector3d>& QuadrupedRobot::getFeetPositionInBase() const
{
  return base_X_foot_;
}

Eigen::Vector3d& QuadrupedRobot::getFootPositionInBase(const std::string& name)
{
  return base_X_foot_[name];
}

const std::map<std::string,Eigen::Affine3d>& QuadrupedRobot::getFeetPoseInWorld() const
{
  return world_T_foot_;
}

Eigen::Affine3d& QuadrupedRobot::getFootPoseInWorld(const std::string& name)
{
  return world_T_foot_[name];
}

const std::map<std::string,Eigen::Affine3d>& QuadrupedRobot::getFeetPoseInBase() const
{
  return base_T_foot_;
}

Eigen::Affine3d& QuadrupedRobot::getFootPoseInBase(const std::string& name)
{
  return base_T_foot_[name];
}

const double& QuadrupedRobot::getHfYawInWorld() const
{
  return yaw_base_;
}

const Eigen::Matrix3d& QuadrupedRobot::getBaseRotationInWorld() const
{
  return world_R_base_;
}

const Eigen::Matrix3d& QuadrupedRobot::getBaseRotationInHf() const
{
  return hf_R_base_;
}

const Eigen::Matrix3d& QuadrupedRobot::getHfRotationInWorld() const
{
  return world_R_hf_;
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

const std::vector<int>& QuadrupedRobot::getLimbJointsIds(const std::string& limb_name)
{
  return joint_limb_idx_[limb_name];
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

void QuadrupedRobot::getFloatingBasePositionInertia(Eigen::Matrix3d& M)
{
  getInertiaMatrix(tmp_M_);
  M = tmp_M_.block(3,3,3,3);
}

void QuadrupedRobot::getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M)
{
  getInertiaMatrix(tmp_M_);
  int n = static_cast<int>(joint_limb_idx_[limb_name].size());
  int idx = joint_limb_idx_[limb_name][0];
  M = tmp_M_.block(idx,idx,n,n);
}

void QuadrupedRobot::getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi)
{
  getInertiaMatrix(tmp_M_);
  tmp_Mi_.setZero();
  tmp_Mi_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,tmp_M_.rows()-FLOATING_BASE_DOFS,tmp_M_.cols()-FLOATING_BASE_DOFS)
      = tmp_M_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,tmp_M_.rows()-FLOATING_BASE_DOFS,tmp_M_.cols()-FLOATING_BASE_DOFS).inverse();
  int n = static_cast<int>(joint_limb_idx_[limb_name].size());
  int idx = joint_limb_idx_[limb_name].at(0);
  Mi = tmp_Mi_.block(idx,idx,n,n);
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
