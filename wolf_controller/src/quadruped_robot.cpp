/**
 * @file robot_model.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief WholeBody Controller.
 *
 * @see todo.git
 */

#include <wolf_controller/quadruped_robot.h>
#include <wolf_controller/utils.h>
#include <wolf_controller/geometry.h>
#include <stdexcept>

using namespace XBot;

namespace wolf_controller {

std::string enumToString(QuadrupedRobot::robot_states_t state)
{
  std::string ret = "NONE";
  switch (state)
  {

  case QuadrupedRobot::robot_states_t::IDLE:
    ret = "IDLE";
    break;

  case QuadrupedRobot::robot_states_t::INIT:
    ret = "INIT";
    break;

  case QuadrupedRobot::robot_states_t::WALKING:
    ret = "WALKING";
    break;

  case QuadrupedRobot::robot_states_t::MANIPULATION:
    ret = "MANIPULATION";
    break;

  case QuadrupedRobot::robot_states_t::ANOMALY:
    ret = "ANOMALY";
    break;

  case QuadrupedRobot::robot_states_t::STANDING_UP:
    ret = "STANDING_UP";
    break;

  case QuadrupedRobot::robot_states_t::STANDING_DOWN:
    ret = "STANDING_DOWN";
    break;

  case QuadrupedRobot::robot_states_t::RESET:
    ret = "RESET";
    break;
  };

  return ret;
}

QuadrupedRobot::QuadrupedRobot(const std::string& urdf, const std::string& srdf)
  :ModelInterfaceRBDL(), robot_state_(robot_states_t::IDLE), robot_state_prev_(robot_states_t::IDLE)
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
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"ROS-Control joints order: " << joint_names_[joint_names_.size()-1]);
    }
  }

  // Initialize the virtual model
  if(!RigidBodyDynamics::Addons::URDFReadFromString(getUrdfString().c_str(), &virtual_model_, isFloatingBase(), false))
      throw std::runtime_error("Can not initialize virtual model");

  const srdf_advr::Model& srdf_model = getSrdf();

  for(unsigned int i=0;i < srdf_model.getGroups().size(); i++)
  {
    const auto& chains = srdf_model.getGroups()[i].chains_;
    const auto& joints = srdf_model.getGroups()[i].joints_;
    const auto& links  = srdf_model.getGroups()[i].links_;
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
    // Parse the base link name from the SRDF file
    if(srdf_model.getGroups()[i].name_.find("base") != std::string::npos)
    {
      if(links.size()==1)
        base_name_ = links[0];
      else
        throw std::runtime_error("There can be only one base defined in the SRDF file!");
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
  if(base_name_.empty())
    throw std::runtime_error("Base can not be empty! Check the SRDF file!");

  for(unsigned int i=0;i<leg_names_.size();i++)
  {
    for(unsigned int j=0;j<joint_legs_[leg_names_[i]].size();j++)
    {
      std::string current_joint_name = joint_legs_[leg_names_[i]].at(j);
      int idx = joint_idx_[current_joint_name];
      joint_limb_idx_[leg_names_[i]].push_back(idx);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,leg_names_[i] << " " << joint_legs_[leg_names_[i]][j] << " " << idx);
    }
  }

  for(unsigned int i=0;i<arm_names_.size();i++)
  {
    for(unsigned int j=0;j<joint_arms_[arm_names_[i]].size();j++)
    {
      std::string current_joint_name = joint_arms_[arm_names_[i]].at(j);
      int idx = joint_idx_[current_joint_name];
      joint_limb_idx_[arm_names_[i]].push_back(idx);
      ROS_DEBUG_STREAM_NAMED(CLASS_NAME,arm_names_[i] << " " << joint_arms_[arm_names_[i]][j] << " " << idx);
    }
  }

  for(unsigned int i=0;i<hip_names_.size();i++)
    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Hip names: "<<hip_names_[i]);

  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Base name: "<<base_name_);

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
  getPose(hip_names_[0],base_name_,pose_lf);
  getPose(hip_names_[1],base_name_,pose_lh);
  getPose(hip_names_[2],base_name_,pose_rf);
  getPose(hip_names_[3],base_name_,pose_rh);
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
  for(unsigned int i=0;i<ee_names_.size();i++)
  {
    world_X_ee_[ee_names_[i]] = Eigen::Vector3d::Zero();
    base_X_ee_[ee_names_[i]] = Eigen::Vector3d::Zero();
    world_T_ee_[ee_names_[i]] = Eigen::Affine3d::Identity();
    base_T_ee_[ee_names_[i]] = Eigen::Affine3d::Identity();
  }

  // Get inertias
  getInertiaMatrix(tmp_M_);
  getInertiaInverse(tmp_Mi_);

  // Get limits
  getEffortLimits(tau_max_);
  getVelocityLimits(qdot_max_);
  getJointLimits(q_min_, q_max_);

  tau_max_.head(6).setZero();

  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Position limits set to: min=["<<q_min_.transpose()<<"]"<< std::endl<<"max=["<<q_max_.transpose()<<"]");
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Velocity limits set to: max=["<<qdot_max_.transpose()<<"]");
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Effort limits set to: max=["<<tau_max_.transpose()<<"]");

  tmp_jacobian_.setZero(6, virtual_model_.dof_count);

  // Get home positions
  getRobotState("standup", q_stand_up_);
  if(!checkJointLimits(q_stand_up_))
    throw std::runtime_error("stand up joint positions are out of the limits! Check the SRDF file!");
  getRobotState("standdown", q_stand_down_);
  if(!checkJointLimits(q_stand_down_))
    throw std::runtime_error("stand down joint positions are out of the limits! Check the SRDF file!");

  // Define default heights
  stand_up_height_ = 0.0;
  Eigen::Affine3d pose;
  for(unsigned int i=0;i<foot_names_.size();i++)
  {
    getPose(q_stand_up_,foot_names_[i],base_name_,pose);
    stand_up_height_ = pose.translation().z() + stand_up_height_;
  }
  stand_up_height_ =  -stand_up_height_/N_LEGS;

  stand_down_height_ = 0.0;
  for(unsigned int i=0;i<foot_names_.size();i++)
  {
    getPose(q_stand_down_,foot_names_[i],base_name_,pose);
    stand_down_height_ = pose.translation().z() + stand_down_height_;
  }
  stand_down_height_ =  -stand_down_height_/N_LEGS;
}

bool QuadrupedRobot::getPose(const Eigen::VectorXd& q, const std::string& source_frame, Eigen::Affine3d& pose)
{

    int body_id = linkId(source_frame);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << source_frame << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    tmp_vector3d_.setZero();

    tmp_matrix3d_ = RigidBodyDynamics::CalcBodyWorldOrientation(virtual_model_,
                                                                q,
                                                                body_id,
                                                                true);

    tmp_vector3d_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(virtual_model_,
                                                                 q,
                                                                 body_id,
                                                                 tmp_vector3d_,
                                                                 true);

    tmp_matrix3d_.transposeInPlace();

    pose.linear() = tmp_matrix3d_;
    pose.translation() = tmp_vector3d_;

    return true;
}

bool QuadrupedRobot::getJacobian(const Eigen::VectorXd &q, const std::string &link_name, Eigen::MatrixXd &J)
{
    int body_id = linkId(link_name);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    tmp_jacobian_.setZero();
    tmp_vector3d_.setZero();

    RigidBodyDynamics::CalcPointJacobian6D(virtual_model_, q, body_id, tmp_vector3d_, tmp_jacobian_, true);

    J.resize(6, virtual_model_.dof_count);

    J.topRows(3) = tmp_jacobian_.bottomRows(3);
    J.bottomRows(3) = tmp_jacobian_.topRows(3);

    return true;

}

bool QuadrupedRobot::getJacobian(const Eigen::VectorXd &q, const std::string &link_name, const std::string &target_frame, Eigen::MatrixXd &J)
{
    bool success = getJacobian(q,link_name, J);
    success = getOrientation(target_frame, tmp_kdl_rotation_) && success;
    tmp_kdl_jacobian_.data = J;
    tmp_kdl_jacobian_.changeBase(tmp_kdl_rotation_.Inverse());
    J = tmp_kdl_jacobian_.data;
    return success;
}

//void QuadrupedRobot::getInertiaMatrix(const Eigen::VectorXd &q, Eigen::MatrixXd &M) const
//{
//    M.setZero(virtual_model_.dof_count, virtual_model_.dof_count);
//    RigidBodyDynamics::CompositeRigidBodyAlgorithm(virtual_model_, q, M, false);
//}

bool QuadrupedRobot::getPose(const Eigen::VectorXd& q, const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose)
{
    bool ret = true;
    ret = getPose(q, source_frame, tmp_affine3d_) && ret;
    ret = getPose(q, target_frame, tmp_affine3d_1_) && ret;

    if(!ret)
    {
        return false;
    }

    pose = tmp_affine3d_1_.inverse() * tmp_affine3d_;

    return true;
}

bool QuadrupedRobot::clampJointPositions(Eigen::VectorXd &q)
{
    assert(q.size() == q_min_.size());
    bool violated_limits = false;
    for(unsigned int i=0;i<q.size();i++)// Maybe I should skip the FB...
    {
        if(q(i)<q_min_(i))
        {
            q(i) = q_min_(i);
            ROS_DEBUG_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Joint("<<wolf_controller::_dof_names[i]<<") violates the minimum POSITION limit of "<<q_min_(i));
            violated_limits = true;
        } else if(q(i)>q_max_(i))
        {
            q(i) = q_max_(i);
            ROS_DEBUG_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Joint("<<wolf_controller::_dof_names[i]<<") violates the maximum POSITION limit of "<<q_max_(i));
            violated_limits = true;
        }
    }
    return violated_limits;
}

bool QuadrupedRobot::clampJointEfforts(Eigen::VectorXd &tau)
{
    assert(tau.size() == tau_max_.size());
    bool violated_limits = false;
    for(unsigned int i=0;i<tau.size();i++)
    {
        if(std::abs(tau(i))>tau_max_(i)+EPS)
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Joint("<<wolf_controller::_dof_names[i]<<") violates the maximum EFFORT limit of "<<tau_max_(i));
            violated_limits = true;
        }
    }
    return violated_limits;
}

bool QuadrupedRobot::clampJointVelocities(Eigen::VectorXd &qdot)
{
    assert(qdot.size() == qdot_max_.size());
    bool violated_limits = false;
    for(unsigned int i=0;i<qdot_max_.size();i++)
    {
        if(std::abs(qdot(i))>qdot_max_(i))
        {
            qdot(i) = qdot_max_(i);
            ROS_DEBUG_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Joint("<<wolf_controller::_dof_names[i]<<") violates the maximum VELOCITY limit of "<<qdot_max_(i));
            violated_limits = true;
        }
    }
    return violated_limits;
}

std::vector<bool> QuadrupedRobot::checkJointVelocities(Eigen::VectorXd &qdot)
{
    assert(qdot.size() == qdot_max_.size());
    std::vector<bool> violated_limits(qdot.size());
    for(unsigned int i=0;i<qdot_max_.size();i++)
    {
        if(std::abs(qdot(i))>qdot_max_(i))
            violated_limits[i] = true;
        else
          violated_limits[i] = false;
    }
    return violated_limits;
}

bool QuadrupedRobot::update(bool update_position, bool update_velocity, bool update_desired_acceleration)
{
  ROS_DEBUG_NAMED(CLASS_NAME,"update");

  // Internal update
  bool res = ModelInterface::update(update_position,update_velocity,update_desired_acceleration);

  // Update the transformations between world, base and horizontal frame
  getPose(base_name_,world_T_base_);
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_T_base.translation()" << world_T_base_.translation());
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_T_base.linear()" << world_T_base_.linear());
  world_R_hf_ = Eigen::Matrix3d::Identity();
  yaw_base_   = std::atan2(world_T_base_.linear()(1,0),world_T_base_.linear()(0,0));
  world_R_hf_ = Eigen::AngleAxisd(yaw_base_,Eigen::Vector3d::UnitZ());
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"yaw_base" << yaw_base_);
  ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_R_hf" << world_R_hf_);
  world_R_base_ = world_T_base_.linear();
  rotToRpy(world_R_base_,world_RPY_base_);
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
    getPose(foot_names_[i],base_name_,base_T_foot_[foot_names_[i]]);
    base_X_foot_[foot_names_[i]] = base_T_foot_[foot_names_[i]].translation();
  }

  for(unsigned int i=0; i<ee_names_.size(); i++)
  {
    // Arms position in world
    getPose(ee_names_[i],world_T_ee_[ee_names_[i]]);
    world_X_ee_[ee_names_[i]] = world_T_ee_[ee_names_[i]].translation();
    // Arms position in base/trunk
    getPose(ee_names_[i],base_name_,base_T_ee_[ee_names_[i]]);
    base_X_ee_[ee_names_[i]] = base_T_ee_[ee_names_[i]].translation();
  }

  return res;
}

const std::map<std::string,Eigen::Vector3d>& QuadrupedRobot::getEndEffectorsPositionInWorld() const
{
  return world_X_ee_;
}

Eigen::Vector3d& QuadrupedRobot::getEndEffectorPositionInWorld(const std::string& name)
{
  return world_X_ee_[name];
}

const std::map<std::string,Eigen::Vector3d>& QuadrupedRobot::getEndEffectorsPositionInBase() const
{
  return base_X_ee_;
}

Eigen::Vector3d& QuadrupedRobot::getEndEffectorPositionInBase(const std::string& name)
{
  return base_X_ee_[name];
}

const std::map<std::string,Eigen::Affine3d>& QuadrupedRobot::getEndEffectorsPoseInWorld() const
{
  return world_T_ee_;
}

const std::map<std::string,Eigen::Affine3d>& QuadrupedRobot::getEndEffectorsPoseInBase() const
{
  return base_T_ee_;
}

Eigen::Affine3d& QuadrupedRobot::getEndEffectorPoseInWorld(const std::string& name)
{
  return world_T_ee_[name];
}

Eigen::Affine3d& QuadrupedRobot::getEndEffectorPoseInBase(const std::string& name)
{
  return base_T_ee_[name];
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

const Eigen::Vector3d &QuadrupedRobot::getBaseRotationInWorldRPY() const
{
  return world_RPY_base_;
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

const std::vector<std::string>& QuadrupedRobot::getLegNames() const
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

const std::vector<std::string>& QuadrupedRobot::getArmNames() const
{
  return arm_names_;
}

const std::vector<std::string>& QuadrupedRobot::getEndEffectorNames() const
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

const std::string &QuadrupedRobot::getBaseLinkName() const
{
  return base_name_;
}

const std::vector<unsigned int>& QuadrupedRobot::getLimbJointsIds(const std::string& limb_name)
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
  XBot::ModelInterfaceRBDL::getInertiaMatrix(tmp_M_);
  M = tmp_M_.block(0,0,3,3);
}

void QuadrupedRobot::getFloatingBaseOrientationInertia(Eigen::Matrix3d& M)
{
  XBot::ModelInterfaceRBDL::getInertiaMatrix(tmp_M_);
  M = tmp_M_.block(3,3,3,3);
}

void QuadrupedRobot::getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M)
{
  XBot::ModelInterfaceRBDL::getInertiaMatrix(tmp_M_);
  int n = static_cast<int>(joint_limb_idx_[limb_name].size());
  int idx = joint_limb_idx_[limb_name][0];
  M = tmp_M_.block(idx,idx,n,n);
}

void QuadrupedRobot::getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi)
{
  XBot::ModelInterfaceRBDL::getInertiaMatrix(tmp_M_);
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

QuadrupedRobot::robot_states_t QuadrupedRobot::getPreviousState()
{
  return robot_state_prev_;
}

bool QuadrupedRobot::setState(QuadrupedRobot::robot_states_t state)
{
  if(state != robot_state_)
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Change state to "<<enumToString(state));
  robot_state_prev_.store(robot_state_.load());
  robot_state_ = state;
  return true;
}

double QuadrupedRobot::getCurrentHeight()
{
  getFloatingBasePose(tmp_affine3d_);
  return tmp_affine3d_.translation().z();
}

const Eigen::VectorXd& QuadrupedRobot::getStandUpJointPostion()
{
  return q_stand_up_;
}

const Eigen::VectorXd& QuadrupedRobot::getStandDownJointPostion()
{
  return q_stand_down_;
}

const double &QuadrupedRobot::getStandUpHeight()
{
  return stand_up_height_;
}

const double &QuadrupedRobot::getStandDownHeight()
{
  return stand_down_height_;
}

const std::string& QuadrupedRobot::getImuSensorName() const
{
  return imu_name_;
}


};
