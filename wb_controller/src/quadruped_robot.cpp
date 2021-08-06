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
  model_imp_ = XBot::ModelInterface::getModel(opt);

  _dof_names = model_imp_->getEnabledJointNames();

  n_arms_ = model_imp_->arms();
  n_legs_ = model_imp_->legs();
  std::vector<int> actuated_joints = model_imp_->getEnabledJointId();

  // Load the joint names
  for(unsigned int i=0;i<actuated_joints.size();i++)
  {
      if(actuated_joints[i]>0) // Filter out the floating base joints
        joint_names_.push_back(model_imp_->getJointByID(actuated_joints[i])->getJointName());
  }

  if(n_legs_ != N_LEGS)
  {
    throw std::runtime_error("Wrong number of legs, check the SRDF file!");
  }
  if(n_arms_ > N_ARMS)
  {
    throw std::runtime_error("Wrong number of arms, check the SRDF file!");
  } 

  const srdf_advr::Model& srdf_model = model_imp_->getSrdf();
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
  limbs = model_imp_->getChainNames();

  for(unsigned int i = 0; i < limbs.size(); i++) // Remove virtual_chain
      if(limbs[i].find("virtual_chain") == std::string::npos)
          limb_names_.push_back(limbs[i]);

  contact_names_ = foot_names_;
  contact_names_.insert( contact_names_.end(), arm_names_.begin(), arm_names_.end() );

  // Calculate approx base length and width based on the hip positions
  // Hips order: "lf","lh","rf","rh"
  Eigen::Affine3d pose_lf, pose_lh, pose_rf, pose_rh;
  model_imp_->getPose(hip_names_[0],BASE_LINK_FRAME_NAME,pose_lf);
  model_imp_->getPose(hip_names_[1],BASE_LINK_FRAME_NAME,pose_lh);
  model_imp_->getPose(hip_names_[2],BASE_LINK_FRAME_NAME,pose_rf);
  model_imp_->getPose(hip_names_[3],BASE_LINK_FRAME_NAME,pose_rh);
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

XBot::ModelInterface::Ptr QuadrupedRobot::getModelImp()
{
  return model_imp_;
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
  model_imp_->getInertiaMatrix(M_);
  Ifb_ = M_.block(3,3,3,3);
  return Ifb_;
}

QuadrupedRobot::robot_states_t QuadrupedRobot::getRobotState()
{
  return robot_state_;
}

bool QuadrupedRobot::setRobotState(QuadrupedRobot::robot_states_t robot_state)
{
  robot_state_ = robot_state;
  return true;
}

bool QuadrupedRobot::getVelocityTwist(const std::string &link_name, KDL::Twist &velocity) const
{
  return model_imp_->getVelocityTwist(link_name,velocity);
}

int QuadrupedRobot::getLinkID(const std::string &link_name) const
{
  return model_imp_->getLinkID(link_name);
}

void QuadrupedRobot::computeInverseDynamics(Eigen::VectorXd &tau) const
{
  model_imp_->computeInverseDynamics(tau);
}

void QuadrupedRobot::computeNonlinearTerm(Eigen::VectorXd &n) const
{
  model_imp_->computeNonlinearTerm(n);
}

void QuadrupedRobot::computeGravityCompensation(Eigen::VectorXd &g) const
{
  model_imp_->computeGravityCompensation(g);
}

void QuadrupedRobot::getInertiaMatrix(Eigen::MatrixXd &M) const
{
  model_imp_->getInertiaMatrix(M);
}

void QuadrupedRobot::setGravity(const KDL::Vector &gravity)
{
  model_imp_->setGravity(gravity);
}

void QuadrupedRobot::getGravity(KDL::Vector &gravity) const
{
  model_imp_->getGravity(gravity);
}

double QuadrupedRobot::getMass() const
{
  return model_imp_->getMass();
}

void QuadrupedRobot::getCentroidalMomentum(Eigen::Vector6d &centroidal_momentum) const
{
  model_imp_->getCentroidalMomentum(centroidal_momentum);
}

void QuadrupedRobot::getCOMAcceleration(KDL::Vector &acceleration) const
{
  return model_imp_->getCOMAcceleration(acceleration);
}

void QuadrupedRobot::getCOMVelocity(KDL::Vector &velocity) const
{
  return model_imp_->getCOMVelocity(velocity);
}

void QuadrupedRobot::getCOM(KDL::Vector &com_position) const
{
  return model_imp_->getCOM(com_position);
}

bool QuadrupedRobot::computeRelativeJdotQdot(const std::string &target_link_name, const std::string &base_link_name, KDL::Twist &jdotqdot) const
{
  return model_imp_->computeRelativeJdotQdot(target_link_name,base_link_name,jdotqdot);
}

bool QuadrupedRobot::getPointAcceleration(const std::string &link_name, const KDL::Vector &point, KDL::Vector &acceleration) const
{
  return model_imp_->getPointAcceleration(link_name,point,acceleration);
}

bool QuadrupedRobot::getRelativeAccelerationTwist(const std::string &link_name, const std::string &base_link_name, KDL::Twist &acceleration) const
{
  return model_imp_->getRelativeAccelerationTwist(link_name,base_link_name,acceleration);
}

bool QuadrupedRobot::getAccelerationTwist(const std::string &link_name, KDL::Twist &acceleration) const
{
 return model_imp_->getAccelerationTwist(link_name,acceleration);
}

bool QuadrupedRobot::getJacobian(const std::string &link_name, const KDL::Vector &reference_point, KDL::Jacobian &J) const
{
  return model_imp_->getJacobian(link_name,reference_point,J);
}

bool QuadrupedRobot::getPose(const std::string &source_frame, KDL::Frame &pose) const
{
  return model_imp_->getPose(source_frame,pose);
}

bool QuadrupedRobot::getFloatingBaseLink(std::string &floating_base_link) const
{
  return model_imp_->getFloatingBaseLink(floating_base_link);
}

bool QuadrupedRobot::setFloatingBaseTwist(const KDL::Twist &floating_base_twist)
{
  return model_imp_->setFloatingBaseTwist(floating_base_twist);
}

bool QuadrupedRobot::setFloatingBasePose(const KDL::Frame &floating_base_pose)
{
  return model_imp_->setFloatingBasePose(floating_base_pose);
}

void QuadrupedRobot::getModelOrderedJoints(std::vector<std::string> &joint_name) const
{
  model_imp_->getModelOrderedJoints(joint_name);
}

bool QuadrupedRobot::update_internal(bool update_position, bool update_velocity, bool update_desired_acceleration)
{
  return model_imp_->update_internal(update_position,update_velocity,update_desired_acceleration);
}

bool QuadrupedRobot::init_model(const ConfigOptions &config)
{
  return model_imp_->init_model(config);
}


};
