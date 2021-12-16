#include <wb_controller/id_problem.h>
#include <OpenSoT/utils/Affine.h>
#include <wb_controller/utils.h>

using namespace OpenSoT;

namespace wb_controller {

IDProblem::IDProblem(ros::NodeHandle& nh, QuadrupedRobot::Ptr model, const double& dt):
  model_(model)
{

  foot_names_          = model_->getFootNames();
  ee_names_            = model_->getEndEffectorNames();
  contact_names_       = model_->getContactNames();
  current_robot_state_ = model_->getState();

  //
  //  This utility internally creates the right variables which later we will use to
  //  create all the tasks and constraints
  //
  id_ = std::make_shared<OpenSoT::utils::InverseDynamics>(foot_names_, *model_, OpenSoT::utils::InverseDynamics::CONTACT_MODEL::POINT_CONTACT);

  //
  // Here we create all the tasks
  //   --------------------------
  for(unsigned int i=0; i<foot_names_.size(); i++)
  {

    feet_[foot_names_[i]] = std::make_shared<Cartesian>(nh,foot_names_[i], *model_, foot_names_[i],
                                                        model_->getBaseLinkName(), id_->getJointsAccelerationAffine());
    feet_[foot_names_[i]]->setLambda(0.,0.);
    feet_[foot_names_[i]]->setWeightIsDiagonalFlag(true);
    feet_[foot_names_[i]]->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
    feet_[foot_names_[i]]->OPTIONS.set_ext_lambda = false;
    feet_[foot_names_[i]]->loadParams();
    feet_[foot_names_[i]]->registerReconfigurableVariables();
  }
  //   --------------------------
  for(unsigned int i=0; i<ee_names_.size(); i++)
  {
    arms_[ee_names_[i]] = std::make_shared<Cartesian>(nh,ee_names_[i], *model_, ee_names_[i],
                                                      model_->getBaseLinkName(), id_->getJointsAccelerationAffine());
    arms_[ee_names_[i]]->setLambda(1.,1.);
    arms_[ee_names_[i]]->setWeightIsDiagonalFlag(true);
    arms_[ee_names_[i]]->setGainType(OpenSoT::tasks::acceleration::GainType::Acceleration);
    arms_[ee_names_[i]]->OPTIONS.set_ext_reference = true;
    arms_[ee_names_[i]]->loadParams();
    arms_[ee_names_[i]]->registerReconfigurableVariables();
  }
  //   --------------------------
  angular_momentum_ = std::make_shared<AngularMomentum>(nh,*model_,id_->getJointsAccelerationAffine());
  angular_momentum_->setLambda(0.);
  angular_momentum_->setWeightIsDiagonalFlag(true);
  angular_momentum_->setReference(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  angular_momentum_->setMomentumGain(Eigen::Matrix3d::Identity());
  angular_momentum_->loadParams();
  angular_momentum_->registerReconfigurableVariables();
  //   --------------------------
  waistRPY_ = std::make_shared<Cartesian>(nh,"waistRPY", *model_, model_->getBaseLinkName(),
                                          WORLD_FRAME_NAME, id_->getJointsAccelerationAffine());
  waistRPY_->setLambda(1.,1.);
  waistRPY_->setWeightIsDiagonalFlag(true);
  waistRPY_->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
  waistRPY_->loadParams();
  waistRPY_->registerReconfigurableVariables();
  //   --------------------------
  waistZ_ = std::make_shared<Cartesian>(nh,"waistZ", *model_, model_->getBaseLinkName(),
                                        WORLD_FRAME_NAME, id_->getJointsAccelerationAffine());
  waistZ_->setLambda(1.,1.);
  waistZ_->setWeightIsDiagonalFlag(true);
  waistZ_->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
  waistZ_->loadParams();
  waistZ_->registerReconfigurableVariables();
  //   --------------------------
  postural_ = std::make_shared<Postural>(nh,*model_, id_->getJointsAccelerationAffine());
  postural_->setLambda(1.,1.);
  postural_->setWeightIsDiagonalFlag(true);
  postural_->loadParams();
  postural_->registerReconfigurableVariables();
  //   --------------------------
  com_ = std::make_shared<CoM>(nh,*model_, id_->getJointsAccelerationAffine());
  com_->setLambda(1.,1.);
  com_->setWeightIsDiagonalFlag(true);
  com_->loadParams();
  com_->registerReconfigurableVariables();

  //
  // Here we create the constraints & bounds
  //
  //   --------------------------
  OpenSoT::constraints::force::FrictionCones::friction_cones fcs;
  fc_.second = 1.0; // mu
  fc_.first.setIdentity();
  for(unsigned int i = 0; i < foot_names_.size(); i++)
    fcs.push_back(fc_);
  friction_cones_ = std::make_shared<OpenSoT::constraints::force::FrictionCones>(foot_names_,id_->getContactsWrenchAffine(),*model_,fcs);
  //   --------------------------
  x_force_lower_lim_ = -2000;
  y_force_lower_lim_ = -2000;
  z_force_lower_lim_ = 0.1*GRAVITY*(model_->getMass()/N_LEGS);
  wrench_upper_lims_<<2000,2000,2000,Eigen::Vector3d::Zero();
  wrench_lower_lims_<<x_force_lower_lim_,y_force_lower_lim_,z_force_lower_lim_,Eigen::Vector3d::Zero();
  wrenches_lims_ = std::make_shared<OpenSoT::constraints::force::WrenchesLimits>(
        foot_names_, wrench_lower_lims_, wrench_upper_lims_,id_->getContactsWrenchAffine());
  //   --------------------------
  Eigen::VectorXd tau_max;
  model_->getEffortLimits(tau_max);
  tau_max.head(FLOATING_BASE_DOFS).setZero();
  torque_lims_ = std::make_shared<OpenSoT::constraints::acceleration::TorqueLimits>(*model_,id_->getJointsAccelerationAffine(),id_->getContactsWrenchAffine(),foot_names_,tau_max);
  //   --------------------------
  //Eigen::VectorXd q_max, q_min, q_home, qddot_max;
  //Eigen::MatrixXd M;
  //model_->getJointLimits(q_min,q_max);
  //q_min.head(FLOATING_BASE_DOFS) = Eigen::Vector6d::Ones() * -10000.0;
  //q_max.head(FLOATING_BASE_DOFS) = Eigen::Vector6d::Ones() *  10000.0;
  //model_->getRobotState("home",q_home);
  ////model_->getInertiaInverseTimesVector(tau_max,qddot_max);
  //model_->setJointPosition(q_home);
  //model_->update();
  //model_->getInertiaMatrix(M);
  //qddot_max = M.inverse() * tau_max;
  //qddot_max.head(FLOATING_BASE_DOFS) = Eigen::Vector6d::Ones() * 10000.0;
  //for(unsigned int i=0; i<qddot_max.size(); i++)
  //    qddot_max(i) = std::abs(qddot_max(i)); // The acceleration limits have to be positive
  //q_lims_ = std::make_shared<OpenSoT::constraints::acceleration::JointLimits>(*model_,id_->getJointsAccelerationAffine(),q_max,q_min,qddot_max,dt);

  //
  // Here we create some indices for the subtask definitions
  //
  std::list<unsigned int> id_XYZ   = {0,1,2}; //xyz
  std::list<unsigned int> id_XY    = {0,1};   //xy
  std::list<unsigned int> id_Z     = {2};     //z
  std::list<unsigned int> id_RPY   = {3,4,5}; //r,p,y
  std::list<unsigned int> id_limbs;
  id_limbs.resize(postural_->getTaskSize()-FLOATING_BASE_DOFS);
  std::list<unsigned int>::iterator it;
  unsigned int idx = FLOATING_BASE_DOFS;
  for (it = id_limbs.begin(); it != id_limbs.end(); ++it)
  {
      *it = idx;
      idx++;
  }

  //
  // Here we create the stack
  //
  OpenSoT::tasks::Aggregated::Ptr feet_aggregated, arm_aggregated;//, arm_aggregated_weighted;
  feet_aggregated = std::make_shared<OpenSoT::tasks::Aggregated>(feet_[foot_names_[0]]%id_XYZ,feet_[foot_names_[0]]->getXSize());
  for(unsigned int i=1;i<foot_names_.size();i++)
    feet_aggregated = feet_aggregated + feet_[foot_names_[i]]%id_XYZ;

  stack_ /= (feet_aggregated +waistRPY_%id_RPY + angular_momentum_ + com_);

  if(ee_names_.size() > 0)
  {
    arm_aggregated = std::make_shared<OpenSoT::tasks::Aggregated>(arms_[ee_names_[0]],arms_[ee_names_[0]]->getXSize());
    //arm_aggregated_weighted = std::make_shared<OpenSoT::tasks::Aggregated>(arms_[ee_names_[0]],arms_[ee_names_[0]]->getXSize());
    if(ee_names_.size() > 1)
    {
      for(unsigned int i=1;i<ee_names_.size();i++)
        arm_aggregated = arm_aggregated + arms_[ee_names_[i]];
       //arm_aggregated_weighted = 50.0 * arm_aggregated%id_XYZ + arm_aggregated%id_RPY;
    }
    stack_->getStack()[0] = arm_aggregated + stack_->getStack()[0];
  }

  stack_ << wrenches_lims_<<torque_lims_<<friction_cones_;

  // Regularization and first update FIXME CLEANUP!
  Eigen::Index n = id_->getSerializer()->getSize();
  Eigen::VectorXd b_reg;
  Eigen::MatrixXd A_reg;
  Eigen::MatrixXd W_reg;
  A_reg = Eigen::MatrixXd::Identity(n,n);
  b_reg = Eigen::VectorXd::Zero(n);
  W_reg = Eigen::MatrixXd::Identity(n,n) * 1e-3;
  unsigned int n_limbs = model_->getNumberArms() + model_->getNumberLegs();
  unsigned int n_forces = 6 * n_limbs;
  regularization_ = std::make_shared<OpenSoT::tasks::GenericTask>("regularization",A_reg,b_reg);
  W_reg.bottomRightCorner(n_forces,n_forces) = W_reg.bottomRightCorner(n_forces,n_forces) * 1e-3;
  regularization_->setWeight(W_reg);
  stack_->setRegularisationTask(regularization_);
  stack_->update(Eigen::VectorXd(1));

  x_.setZero(id_->getSerializer()->getSize());

  qddot_.setZero(model_->getJointNum());
  contact_wrenches_.reserve(contact_names_.size());

  solver_ = std::make_unique<OpenSoT::solvers::iHQP>(stack_->getStack(), stack_->getBounds(),1.0);
  // ,OpenSoT::solvers::solver_back_ends::OSQP);
  // ,OpenSoT::solvers::solver_back_ends::eiQuadProg);
}

IDProblem::~IDProblem()
{
}

void IDProblem::setFrictionConesMu(const double& mu)
{
  assert(mu>=0.0 && mu<=1.0);
  fc_.second = mu;
}

double IDProblem::getFrictionConesMu() const
{
   return fc_.second;
}

void IDProblem::setFrictionConesR(const Eigen::Matrix3d& R)
{
   fc_.first = R;
}

void IDProblem::setFootReference(const std::string& foot_name, const Eigen::Affine3d& pose_ref, const Eigen::Vector6d& vel_ref, const std::string& reference_frame)
{
  if(reference_frame == model_->getBaseLinkName())
    feet_[foot_name]->setReference(pose_ref,vel_ref);
  else if(reference_frame == WORLD_FRAME_NAME)
  {
    tmp_affine3d_.setIdentity();
    tmp_affine3d_ = model_->getBasePoseInWorld().inverse(); // base_T_world
    tmp_vector6d_.setZero();
    tmp_vector3d_ = vel_ref.head(3);
    tmp_vector6d_.head(3) = tmp_affine3d_ * tmp_vector3d_;
    feet_[foot_name]->setReference(tmp_affine3d_*pose_ref,tmp_vector6d_);
  }
  else
    throw std::runtime_error("Wrong reference frame, can not set the foot references!");
}

void IDProblem::setLowerForceBound(const double& x_force,const double& y_force,const double& z_force)
{
  x_force_lower_lim_ = x_force;
  y_force_lower_lim_ = y_force;
  z_force_lower_lim_ = z_force;
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set x force lower lim to: "<<x_force);
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set y force lower lim to: "<<y_force);
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set z force lower lim to: "<<z_force);
}

void IDProblem::setLowerForceBoundX(const double& force)
{
  x_force_lower_lim_ = force;
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set x force lower lim to: "<<force);
}

void IDProblem::setLowerForceBoundY(const double& force)
{
  y_force_lower_lim_ = force;
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set y force lower lim to: "<<force);
}

void IDProblem::setLowerForceBoundZ(const double& force)
{
  z_force_lower_lim_ = force;
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set z force lower lim to: "<<force);
}

void IDProblem::update()
{
  // Update if the robot's state changed
  if(current_robot_state_ != model_->getState())
  {
    current_robot_state_ = model_->getState();

    if(ee_names_.size()>0)
    {
      std::string frame;
      if(current_robot_state_ == QuadrupedRobot::WALKING)
        frame = model_->getBaseLinkName();
      else if (current_robot_state_ == QuadrupedRobot::MANIPULATION)
        frame = WORLD_FRAME_NAME;
      else
        frame = model_->getBaseLinkName();

      for (auto& tmp_map : arms_)
      {
        tmp_map.second->setBaseLink(frame);
        tmp_map.second->update(Eigen::VectorXd(1));
        tmp_map.second->reset();
      }
    }
  }

  // Update the mu and the wrench limits
  wrench_lower_lims_(0) = x_force_lower_lim_;
  wrench_lower_lims_(1) = y_force_lower_lim_;
  wrench_lower_lims_(2) = z_force_lower_lim_;
  for (auto& tmp_map : feet_)
  {
    friction_cones_->getFrictionCone(tmp_map.first)->setFrictionCone(fc_);
    if(!wrenches_lims_->getWrenchLimits(tmp_map.first)->isReleased())
      wrenches_lims_->getWrenchLimits(tmp_map.first)->setWrenchLimits(wrench_lower_lims_,wrench_upper_lims_);
  }

  // Update the problem
  stack_->update(Eigen::VectorXd(1));
}

void IDProblem::publish(const ros::Time& time)
{
  for (auto& tmp_map : feet_)
    tmp_map.second->publish(time);
  for (auto& tmp_map : arms_)
    tmp_map.second->publish(time);
  waistRPY_->publish(time);
  waistZ_->publish(time);
  com_->publish(time);
  postural_->publish(time);
  angular_momentum_->publish(time);
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
  bool res_solv = false;
  bool res_id = false;
  if (solver_)
  {
    update();
    res_solv = solver_->solve(x_);
    if(res_solv)
      res_id = id_->computedTorque(x_, tau, qddot_, contact_wrenches_);
  }

  // Update the costs
#ifdef COMPUTE_COST
  for (auto& tmp_map : feet_)
    tmp_map.second->updateCost(x_);
  for (auto& tmp_map : arms_)
    tmp_map.second->updateCost(x_);
  waistRPY_->updateCost(x_);
  waistZ_->updateCost(x_);
  com_->updateCost(x_);
  postural_->updateCost(x_);
  angular_momentum_->updateCost(x_);
#endif

  return (res_solv && res_id);
}

const std::vector<Eigen::Vector6d>& IDProblem::getContactWrenches() const
{
  return contact_wrenches_;
}

const Eigen::VectorXd& IDProblem::getJointAccelerations() const
{
  return qddot_;
}

void IDProblem::swingWithFoot(const string &foot_name)
{
  //feet_[foot_name]->setActive(false);
  feet_[foot_name]->setLambda(1.,1.);
  wrenches_lims_->getWrenchLimits(foot_name)->releaseContact(true);
  torque_lims_->disableContact(foot_name);
}

void IDProblem::stanceWithFoot(const string &foot_name)
{
  //feet_[foot_name]->setActive(true);
  feet_[foot_name]->setLambda(0.,0.);
  wrenches_lims_->getWrenchLimits(foot_name)->releaseContact(false);
  torque_lims_->enableContact(foot_name);
}

void IDProblem::setWaistReference(const Eigen::Matrix3d& Rot, const double& z)
{
  tmp_affine3d_.setIdentity();
  tmp_affine3d_.linear() = Rot;
  tmp_affine3d_.translation().z() = z;
  waistRPY_->setReference(tmp_affine3d_);
}

void IDProblem::setComReference(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity)
{
  com_->setReference(position,velocity);
}

} // namespace
