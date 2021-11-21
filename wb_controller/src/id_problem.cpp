#include <wb_controller/id_problem.h>
#include <OpenSoT/utils/Affine.h>
#include <wb_controller/utils.h>

using namespace OpenSoT;

namespace wb_controller {

IDProblem::IDProblem(ros::NodeHandle& nh, QuadrupedRobot::Ptr model):
  model_(model),
  current_stack_(stacks_t::NONE)
{

  foot_names_    = model_->getFootNames();
  ee_names_      = model_->getEndEffectorNames();
  contact_names_ = model_->getContactNames();

  //
  //  This utility internally creates the right variables which later we will use to
  //  create all the tasks and constraints
  //
  id_ = std::make_shared<OpenSoT::utils::InverseDynamics>(foot_names_, *model_, OpenSoT::utils::InverseDynamics::CONTACT_MODEL::POINT_CONTACT);

  //
  // Here we create all the tasks
  //   --------------------------
  ROS_INFO_NAMED(CLASS_NAME,"Initialize FOOT tasks");
  for(unsigned int i=0; i<foot_names_.size(); i++)
  {

    feet_[foot_names_[i]] = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>(foot_names_[i], *model_, foot_names_[i],
                                                                                      model_->getBaseLinkName(), id_->getJointsAccelerationAffine());
    feet_[foot_names_[i]]->setLambda(0.,0.);
    feet_[foot_names_[i]]->setWeightIsDiagonalFlag(true);
    feet_[foot_names_[i]]->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
  }
  //   --------------------------
  ROS_INFO_NAMED(CLASS_NAME,"Initialize ARM tasks");
  for(unsigned int i=0; i<ee_names_.size(); i++)
  {
    arms_[ee_names_[i]] = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>(ee_names_[i], *model_, ee_names_[i],
                                                                                    model_->getBaseLinkName(), id_->getJointsAccelerationAffine());
    arms_[ee_names_[i]]->setLambda(1.,1.);
    arms_[ee_names_[i]]->setWeightIsDiagonalFlag(true);
    arms_[ee_names_[i]]->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
  }
  //   --------------------------
  angular_momentum_ = std::make_shared<OpenSoT::tasks::acceleration::AngularMomentum>(*model_,id_->getJointsAccelerationAffine());
  angular_momentum_->setLambda(0.);
  angular_momentum_->setWeightIsDiagonalFlag(true);
  angular_momentum_->setReference(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  angular_momentum_->setMomentumGain(Eigen::Matrix3d::Identity());
  //   --------------------------
  waistRPY_ = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistRPY", *model_, model_->getBaseLinkName(),
                                                                        WORLD_FRAME_NAME, id_->getJointsAccelerationAffine());
  waistRPY_->setLambda(1.,1.);
  waistRPY_->setWeightIsDiagonalFlag(true);
  waistRPY_->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
  //   --------------------------
  waistZ_ = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistZ", *model_, model_->getBaseLinkName(),
                                                                      WORLD_FRAME_NAME, id_->getJointsAccelerationAffine());
  waistZ_->setLambda(1.,1.);
  waistZ_->setWeightIsDiagonalFlag(true);
  waistZ_->setGainType(OpenSoT::tasks::acceleration::GainType::Force);
  //   --------------------------
  postural_ = std::make_shared<OpenSoT::tasks::acceleration::Postural>(*model_, id_->getJointsAccelerationAffine());
  postural_->setLambda(1.,1.);
  postural_->setWeightIsDiagonalFlag(true);
  //   --------------------------
  com_ = std::make_shared<OpenSoT::tasks::acceleration::CoM>(*model_, id_->getJointsAccelerationAffine());
  com_->setLambda(1.,1.);
  com_->setWeightIsDiagonalFlag(true);

  //
  // Here we create the constraints & bounds
  //
  ROS_INFO_NAMED(CLASS_NAME,"Initialize Dynamic Feasibility");
  dynamics_task_ = std::make_shared<OpenSoT::tasks::acceleration::DynamicFeasibility>("dynamics", *model_,
                                                                                      id_->getJointsAccelerationAffine(), id_->getContactsWrenchAffine(), foot_names_);

  dynamics_con_ = std::make_shared<OpenSoT::constraints::TaskToConstraint>(dynamics_task_);

  ROS_INFO_NAMED(CLASS_NAME,"Initialize Friction Cones");
  OpenSoT::constraints::force::FrictionCones::friction_cones fcs;
  fc_.second = 1.0; // mu
  fc_.first.setIdentity();
  for(unsigned int i = 0; i < foot_names_.size(); i++)
    fcs.push_back(fc_);
  friction_cones_ = std::make_shared<OpenSoT::constraints::force::FrictionCones>(foot_names_,id_->getContactsWrenchAffine(),*model_,fcs);

  joint_acceleration_lim_ = 1000.;
  ones_ = Eigen::VectorXd::Ones(model_->getJointNum());

  qddot_lims_ = std::make_shared<OpenSoT::constraints::GenericConstraint>(
        "acc_lims", id_->getJointsAccelerationAffine(), joint_acceleration_lim_ * ones_, -1.0 * joint_acceleration_lim_ * ones_, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

  x_force_lower_lim_ = -2000;
  y_force_lower_lim_ = -2000;
  z_force_lower_lim_ = 0.1*GRAVITY*(model_->getMass()/N_LEGS);

  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Robot's weight is: "<<model_->getMass());


  wrench_upper_lims_<<2000,2000,2000,Eigen::Vector3d::Zero();
  wrench_lower_lims_<<x_force_lower_lim_,y_force_lower_lim_,z_force_lower_lim_,Eigen::Vector3d::Zero();

  ROS_INFO_NAMED(CLASS_NAME,"Initialize Wrench Limits");
  wrenches_lims_ = std::make_shared<OpenSoT::constraints::force::WrenchesLimits>(
        foot_names_, wrench_lower_lims_, wrench_upper_lims_,id_->getContactsWrenchAffine());

  ROS_INFO_NAMED(CLASS_NAME,"Initialize Torque Limits");
  Eigen::VectorXd tau_max;
  model_->getEffortLimits(tau_max);
  tau_max.head(6).setZero();
  torque_lims_ = std::make_shared<OpenSoT::constraints::acceleration::TorqueLimits>(*model_,id_->getJointsAccelerationAffine(),id_->getContactsWrenchAffine(),foot_names_,tau_max);

  std::list<unsigned int> id_XYZ   = {0,1,2}; //xyz
  std::list<unsigned int> id_XY    = {0,1};   //xy
  std::list<unsigned int> id_Z     = {2};     //z
  std::list<unsigned int> id_RPY   = {3,4,5}; //r,p,y
  std::list<unsigned int> id_legs;
  id_legs.resize(postural_->getTaskSize()-FLOATING_BASE_DOFS);
  std::list<unsigned int>::iterator it;
  unsigned int idx = FLOATING_BASE_DOFS;
  for (it = id_legs.begin(); it != id_legs.end(); ++it)
  {
      *it = idx;
      idx++;
  }

  OpenSoT::tasks::Aggregated::Ptr feet_aggregated, arm_aggregated;//, arm_aggregated_weighted;
  feet_aggregated = std::make_shared<OpenSoT::tasks::Aggregated>(feet_[foot_names_[0]]%id_XYZ,feet_[foot_names_[0]]->getXSize());
  for(unsigned int i=1;i<foot_names_.size();i++)
    feet_aggregated = feet_aggregated + feet_[foot_names_[i]]%id_XYZ;

  stacks_[MANIPULATION] = ( (feet_aggregated)
                          / (com_)
                          / (waistRPY_%id_RPY)// + arm_aggregated
                          / (postural_%id_legs)
                          )<<wrenches_lims_<<friction_cones_<<torque_lims_;

  // Original stack, it doesn't work with aliengo e anymal
  //int stack_pos_offset = 2;
  //stacks_[WALKING] = ( (feet_aggregated)
  //                     / (waistRPY_%id_RPY)
  //                     / (postural_ + com_ + angular_momentum_)
  //                     )<<wrenches_lims_<<qddot_lims_<<dynamics_con_<<friction_cones_;

  //int stack_pos_offset = 1;
  //stacks_[WALKING] = ((feet_aggregated)  / ( waistRPY_%id_RPY + waistZ_%id_Z + angular_momentum_ + com_) / (postural_%id_legs)
  //                   )<<wrenches_lims_<<torque_lims_<<friction_cones_;

  int stack_pos_offset = 0;
  stacks_[WALKING] = ((feet_aggregated + waistRPY_%id_RPY + waistZ_%id_Z + angular_momentum_ + com_) / (postural_%id_legs)
                     )<<wrenches_lims_<<torque_lims_<<friction_cones_;

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

    stacks_[MANIPULATION]->getStack()[2] = 30.0 * arm_aggregated + stacks_[MANIPULATION]->getStack()[2];
    auto it = stacks_[WALKING]->getStack().begin() + stack_pos_offset;
    stacks_[WALKING]->getStack().insert(it, arm_aggregated);
  }

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

  for (auto& tmp_map : stacks_)
  {
    tmp_map.second->setRegularisationTask(regularization_);
    tmp_map.second->update(Eigen::VectorXd(1));
  }

  x_.setZero(id_->getSerializer()->getSize());

  qddot_.setZero(model_->getJointNum());
  contact_wrenches_.reserve(contact_names_.size());

  // Add some ROS magic (TO BE MOVED)
  tasks_ros_["waistRPY"] = std::make_shared<CartesianWrapper>(nh,waistRPY_); // WAIST RPY
  tasks_ros_["waistZ"] = std::make_shared<CartesianWrapper>(nh,waistZ_); // WAIST Z
  tasks_ros_["postural"] = std::make_shared<PosturalWrapper>(nh,postural_); // POSTURAL
  tasks_ros_["postural"]->OPTIONS.set_ext_gains = false;
  tasks_ros_["com"] = std::make_shared<ComWrapper>(nh,com_); // CoM

  for(unsigned int i=0; i<foot_names_.size(); i++)
  {
    tasks_ros_[foot_names_[i]] = std::make_shared<CartesianWrapper>(nh,feet_[foot_names_[i]]); // FEET
    tasks_ros_[foot_names_[i]]->OPTIONS.set_ext_lambda = false;
  }
  for(unsigned int i=0; i<ee_names_.size(); i++)
  {
    tasks_ros_[ee_names_[i]] = std::make_shared<CartesianWrapper>(nh,arms_[ee_names_[i]]); // ARMS
    tasks_ros_[ee_names_[i]]->OPTIONS.set_ext_reference = true;
  }

  //selectStack(stacks_t::WALKING);
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

void IDProblem::setJointAccelerationAbsLim(const double& lim)
{
   assert(lim>=0.0);
   joint_acceleration_lim_ = lim;
}

double IDProblem::getJointAccelerationAbsLim()
{
   return joint_acceleration_lim_;
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

void IDProblem::selectStack(const stacks_t& stack)
{

  if(current_stack_ != stack)
  {
    solver_lock_.lock();

    current_stack_ = stack;

    if(ee_names_.size()>0)
    {
      std::string frame;
      if(stack == stacks_t::WALKING)
        frame = model_->getBaseLinkName();
      else if (stack == stacks_t::MANIPULATION)
        frame = WORLD_FRAME_NAME;
      else
        ROS_WARN_NAMED(CLASS_NAME,"Wrong stack!");

      for (auto& tmp_map : arms_)
      {
        tmp_map.second->setBaseLink(frame);
        tmp_map.second->update(Eigen::VectorXd(1));
      }
      //for (unsigned int i=0;i<ee_names_.size();i++)
      //  tasks_ros_[ee_names_[i]]->reset();
    }

    for (auto& tmp_map : tasks_ros_)
      tmp_map.second->reset();

    if(solver_.get()!=nullptr)
      solver_.release();
    solver_ = std::make_unique<OpenSoT::solvers::iHQP>(stacks_[current_stack_]->getStack(), stacks_[current_stack_]->getBounds(),1.0);
    // ,OpenSoT::solvers::solver_back_ends::OSQP);
    // ,OpenSoT::solvers::solver_back_ends::eiQuadProg);
    solver_lock_.unlock();
  }
}

void IDProblem::switchStack()
{
  if(current_stack_ == stacks_t::WALKING)
    selectStack(stacks_t::MANIPULATION);
  else
    selectStack(stacks_t::WALKING);
}

unsigned int IDProblem::getCurrentStack()
{
  return current_stack_;
}

void IDProblem::update()
{
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

  qddot_lims_->setBounds(joint_acceleration_lim_ * ones_,-1.0*joint_acceleration_lim_ * ones_);

  //Update the external lambda/references etc...
  for (auto& tmp_map : tasks_ros_)
    tmp_map.second->update();

  // Update robot state
  if(current_stack_ == stacks_t::WALKING)
    model_->setState(QuadrupedRobot::robot_states_t::WALKING);
  else if (current_stack_ == stacks_t::MANIPULATION)
    model_->setState(QuadrupedRobot::robot_states_t::MANIPULATION);
  else
    model_->setState(QuadrupedRobot::robot_states_t::INIT);

  // Update the problem
  stacks_[current_stack_]->update(Eigen::VectorXd(1));
}

void IDProblem::publish(const ros::Time& time)
{
  for (auto& tmp_map : tasks_ros_)
    tmp_map.second->publish(time);
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
  bool res_solv = false;
  bool res_id = false;
  if (solver_ && solver_lock_.try_lock())
  {
    update();
    res_solv = solver_->solve(x_);
    if(res_solv)
      res_id = id_->computedTorque(x_, tau, qddot_, contact_wrenches_);
    solver_lock_.unlock();
  }

  // Update the costs
#ifdef COMPUTE_COST
  for (auto& tmp_map : tasks_ros_)
    tmp_map.second->computeCost(x_);
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
  waistZ_->setReference(tmp_affine3d_);
}

void IDProblem::setComReference(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity)
{
  com_->setReference(position,velocity);
}

} // namespace
