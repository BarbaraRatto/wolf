#include <wb_controller/id_problem.h>
#include <OpenSoT/utils/Affine.h>
#include <wb_controller/utils.h>

using namespace OpenSoT;

namespace wb_controller {

#define CLASS_NAME "IDProblem"

IDProblem::IDProblem(ros::NodeHandle& nh, QuadrupedRobot::Ptr model):
    model_(model),
    current_stack_(stacks_t::NONE)
{

    foot_names_    = model_->getFootNames();
    arm_names_     = model_->getArmNames();
    contact_names_ = model_->getContactNames();

    // Load some params from the ROS server
    double default_z_lower_force = 20; // [N]
    if (!nh.getParam("default_z_lower_force", default_z_lower_force))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default z lower force given in namespace %s, using a default value of %f.", nh.getNamespace().c_str(),default_z_lower_force);
    }

    //
    //  This utility internally creates the right variables which later we will use to
    //  create all the tasks and constraints
    //
    id_ = std::make_shared<OpenSoT::utils::InverseDynamics>(foot_names_, *model_->getXBotModel());

    //
    // Here we create all the tasks
    //   --------------------------
    ROS_INFO("Initialize FOOT tasks");
    for(unsigned int i=0; i<foot_names_.size(); i++)
    {

        feet_[foot_names_[i]] = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>(foot_names_[i], *model_->getXBotModel(), foot_names_[i],
                                                                               "world", id_->getJointsAccelerationAffine());
        feet_[foot_names_[i]]->setLambda(0.,0.);
        feet_[foot_names_[i]]->setWeightIsDiagonalFlag(true);
    }
    //   --------------------------
    ROS_INFO("Initialize ARM tasks");
    for(unsigned int i=0; i<arm_names_.size(); i++)
    {
        arms_[arm_names_[i]] = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>(arm_names_[i], *model_->getXBotModel(), arm_names_[i],
                                                               "base_link", id_->getJointsAccelerationAffine());
        arms_[arm_names_[i]]->setLambda(1,1);
        arms_[arm_names_[i]]->setWeightIsDiagonalFlag(true);

    }
    //   --------------------------
    angular_momentum_ = std::make_shared<OpenSoT::tasks::acceleration::AngularMomentum>(*model_->getXBotModel(),id_->getJointsAccelerationAffine());
    angular_momentum_->setLambda(0.);
    angular_momentum_->setWeightIsDiagonalFlag(true);
    angular_momentum_->setReference(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    angular_momentum_->setMomentumGain(Eigen::Matrix3d::Identity());
    //   --------------------------
    waistRPY_ = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistRPY", *model_->getXBotModel(), "base_link",
                                                                          "world", id_->getJointsAccelerationAffine());
    waistRPY_->setLambda(1.,1.);
    waistRPY_->setWeightIsDiagonalFlag(true);
    //   --------------------------
    waistZ_ = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistZ", *model_->getXBotModel(), "base_link",
                                                                        "world", id_->getJointsAccelerationAffine());
    waistZ_->setLambda(1.,1.);
    waistZ_->setWeightIsDiagonalFlag(true);
    //   --------------------------
    postural_ = std::make_shared<OpenSoT::tasks::acceleration::Postural>(*model_->getXBotModel(), id_->getJointsAccelerationAffine());
    postural_->setLambda(1.,1.);
    postural_->setWeightIsDiagonalFlag(true);
    //   --------------------------
    com_ = std::make_shared<OpenSoT::tasks::acceleration::CoM>(*model_->getXBotModel(), id_->getJointsAccelerationAffine());
    com_->setLambda(0.,100.);
    com_->setWeightIsDiagonalFlag(true);

    //
    // Here we create the constraints & bounds
    //
    dynamics_task_ = std::make_shared<OpenSoT::tasks::acceleration::DynamicFeasibility>("dynamics", *model_->getXBotModel(),
                                                                          id_->getJointsAccelerationAffine(), id_->getContactsWrenchAffine(), contact_names_);

    dynamics_con_ = std::make_shared<OpenSoT::constraints::TaskToConstraint>(dynamics_task_);


    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    mu_ = 0.7;
    for(unsigned int i = 0; i < foot_names_.size(); i++)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,mu_));
    friction_cones_ = std::make_shared<OpenSoT::constraints::force::FrictionCones>(foot_names_,id_->getContactsWrenchAffine(),*model_->getXBotModel(),mus);

    /// HERE WE SET SOME BOUNDS
    Eigen::VectorXd xmax = 500.*Eigen::VectorXd::Ones(model_->getXBotModel()->getJointNum());
    Eigen::VectorXd xmin = -xmax;

    qddot_lims_ = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                          "acc_lims", id_->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    x_force_lower_lim_ = -2000;
    y_force_lower_lim_ = -2000;
    z_force_lower_lim_ = default_z_lower_force;

    wrench_upper_lims_<<2000,2000,2000,Eigen::Vector3d::Zero();
    wrench_lower_lims_<<x_force_lower_lim_,y_force_lower_lim_,z_force_lower_lim_,Eigen::Vector3d::Zero();

    wrenches_lims_ = std::make_shared<OpenSoT::constraints::force::WrenchesLimits>(
                             contact_names_, wrench_lower_lims_, wrench_upper_lims_,id_->getContactsWrenchAffine());

    std::list<unsigned int> idx_Y  = {0,1};   //xy
    std::list<unsigned int> id_Z   = {2};     //z
    std::list<unsigned int> id_RPY = {3,4,5}; //r,p,y
    std::list<unsigned int> idx_YZ = {0,1,2}; //xyz

    OpenSoT::tasks::Aggregated::Ptr feet_aggregated;
    for(unsigned int i=0;i<foot_names_.size()-1;i++) // This is ok because we always have more than two legs
      feet_aggregated = feet_[foot_names_[i]]%idx_YZ + feet_[foot_names_[i+1]]%idx_YZ;

    stacks_[MANIPULATION] = ( (feet_aggregated)
                            / (com_%idx_Y)
                            / (0.5*waistRPY_%id_RPY + 0.25*waistZ_%id_Z + 50.0*com_%id_Z + angular_momentum_)
                            / (postural_)
                            )<<wrenches_lims_<<qddot_lims_<<dynamics_con_<<friction_cones_;

    stacks_[WALKING] = ( (feet_aggregated)
                       / (waistRPY_%id_RPY)
                       / (postural_ + com_ + angular_momentum_)
                       )<<wrenches_lims_<<qddot_lims_<<dynamics_con_<<friction_cones_;

    if(arm_names_.size()>0)
    {

      OpenSoT::tasks::Aggregated::Ptr arm_aggregated;
      for(unsigned int i=0;i<arm_names_.size()-1;i++)
         arm_aggregated = arms_[arm_names_[i]] + arms_[arm_names_[i+1]];

      // Stack manipolazione con arm wrt world
      //stacks_[MANIPULATION] = ( (feet_aggregated)
      //                        / (com_%idx_Y)
      //                        / (0.5*waistRPY_%id_RPY + 0.25*waistZ_%id_Z + arm_aggregated + 50.0*com_%id_Z + angular_momentum_)
      //                        / (postural_)
      //                        )<<wrenches_lims_<<qddot_lims_<<dynamics_con_<<friction_cones_;
      //
      //// Stack base_link camminata, ok ad alte frequenze
      //stacks_[WALKING] = ( (feet_aggregated)
      //                   / (waistRPY_%id_RPY)
      //                   / (arm_aggregated)
      //                   / (postural_ + com_ + angular_momentum_)
      //                   )<<wrenches_lims_<<qddot_lims_<<dynamics_con_<<friction_cones_;

      stacks_[MANIPULATION]->getStack()[2] = stacks_[MANIPULATION]->getStack()[2] + arm_aggregated;

      auto it = stacks_[WALKING]->getStack().begin() + 2;
      stacks_[WALKING]->getStack().insert(it, arm_aggregated);
      stacks_[WALKING]->getStack()[2] = stacks_[MANIPULATION]->getStack()[2] + arm_aggregated;

    }
    //else
    //{
    //
    //   stacks_[WALKING] = ( (feet_aggregated)
    //                      / (waistRPY_%id_RPY)
    //                      / (postural_ + com_ + angular_momentum_) // Dunno if it really change anything with the angular momentum and the com task
    //                      )<<wrenches_lims_<<qddot_lims_<<dynamics_con_<<friction_cones_;
    //}

    for (auto& tmp_map : stacks_)
        tmp_map.second->update(Eigen::VectorXd(1));

    x_.setZero(id_->getSerializer()->getSize());

    qddot_.setZero(model_->getXBotModel()->getJointNum());
    contact_wrenches_.reserve(contact_names_.size());

    // Add some ROS magic
    tasks_ros_["waistRPY"] = std::make_shared<CartesianWrapper>(nh,waistRPY_); // WAIST RPY
    tasks_ros_["waistZ"] = std::make_shared<CartesianWrapper>(nh,waistZ_); // WAIST Z
    tasks_ros_["postural"] = std::make_shared<PosturalWrapper>(nh,postural_); // POSTURAL
    tasks_ros_["com"] = std::make_shared<ComWrapper>(nh,com_); // CoM
    for(unsigned int i=0; i<foot_names_.size(); i++)
        tasks_ros_[foot_names_[i]] = std::make_shared<CartesianWrapper>(nh,feet_[foot_names_[i]]); // FEET
    for(unsigned int i=0; i<arm_names_.size(); i++)
        tasks_ros_[arm_names_[i]] = std::make_shared<CartesianWrapper>(nh,arms_[arm_names_[i]]); // ARMS

    for (auto& tmp_map : tasks_ros_)
        tmp_map.second->dynamicReconfigureUpdate();

    // Set the callback for the dynamic reconfigure server
    ros::NodeHandle problem_nh("problem");
    server_.reset(new dynamic_reconfigure::Server<wb_controller::problemConfig>(problem_nh));
    server_->setCallback(boost::bind(&IDProblem::dynamicReconfigureCallback, this, _1, _2));

    dynamicReconfigureUpdate();

    selectStack(stacks_t::WALKING);
}

IDProblem::~IDProblem()
{
}

void IDProblem::dynamicReconfigureCallback(wb_controller::problemConfig &config, uint32_t level)
{
    switch(level)
    {
    case 0:
        setFrictionConesMu(config.mu);
        break;
    case 1:
        setLowerForceBound(config.x_force_lower_lim,config.y_force_lower_lim,config.z_force_lower_lim);
        break;
    default:
        break;
    }
}

void IDProblem::dynamicReconfigureUpdate()
{
    // Update the config for dynamic reconfigure
    default_config_.mu = mu_;
    default_config_.x_force_lower_lim = x_force_lower_lim_;
    default_config_.y_force_lower_lim = y_force_lower_lim_;
    default_config_.z_force_lower_lim = z_force_lower_lim_;

    if(server_)
        server_->updateConfig(default_config_);
}

void IDProblem::setFrictionConesMu(const double& mu)
{
    if(mu>=0.0 && mu<=1.0)
    {
        mu_ = mu;
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set mu to: "<<mu);
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Mu has to be between 0 and 1!");
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
    current_stack_ = stack;

    solver_lock_.lock();

    switch (current_stack_)
    {
      case stacks_t::WALKING:
        if(arm_names_.size()>0)
          for (auto& tmp_map : arms_)
          {
            tmp_map.second->setBaseLink("base_link");
            tmp_map.second->update(Eigen::VectorXd(1));
          }
          ROS_INFO_NAMED(CLASS_NAME,"STACK WALKING SELECTED");
          break;
      case stacks_t::MANIPULATION:
        if(arm_names_.size()>0)
          for (auto& tmp_map : arms_)
          {
            tmp_map.second->setBaseLink("world");
            tmp_map.second->update(Eigen::VectorXd(1));
          }
          ROS_INFO_NAMED(CLASS_NAME,"STACK MANIPULATION SELECTED");
          break;
      default:
        ROS_WARN_NAMED(CLASS_NAME,"Wrong stack selected!");
        return;
    };

    for (unsigned int i=0;i<arm_names_.size();i++)
      tasks_ros_[arm_names_[i]]->reset();

    //std::cout << "solver_ = std::make_unique<OpenSoT::solvers::iHQP>" << std::endl;
    if(solver_.get()!=nullptr)
      solver_.release();
    solver_ = std::make_unique<OpenSoT::solvers::iHQP>(stacks_[current_stack_]->getStack(), stacks_[current_stack_]->getBounds(),1e6); //, 1e6);
    // , OpenSoT::solvers::solver_back_ends::OSQP);
    // , OpenSoT::solvers::solver_back_ends::eiQuadProg);
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
        friction_cones_->getFrictionCone(tmp_map.first)->setMu(mu_);
        if(!wrenches_lims_->getWrenchLimits(tmp_map.first)->isReleased())
            wrenches_lims_->getWrenchLimits(tmp_map.first)->setWrenchLimits(wrench_lower_lims_,wrench_upper_lims_);
    }
    // Update the problem
    stacks_[current_stack_]->update(Eigen::VectorXd(1));
}

void IDProblem::setExternalReference(const std::string& task_name)
{
    if(tasks_ros_.count(task_name))
        tasks_ros_[task_name]->updateReference();
}

void IDProblem::publish(const ros::Time& time)
{
    for (auto& tmp_map : tasks_ros_)
        tmp_map.second->publish(time);
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
    update();

    bool res_solv = false;
    bool resid_ = false;
    if (solver_lock_.try_lock())
    {
      res_solv = solver_->solve(x_);
      if(res_solv)
        resid_ = id_->computedTorque(x_, tau, qddot_, contact_wrenches_);
      solver_lock_.unlock();
    }
    return (res_solv && resid_);
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
  feet_[foot_name]->setActive(false);
  wrenches_lims_->getWrenchLimits(foot_name)->releaseContact(true);
}

void IDProblem::stanceWithFoot(const string &foot_name)
{
  feet_[foot_name]->setActive(true);
  wrenches_lims_->getWrenchLimits(foot_name)->releaseContact(false);
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
  com_->setReference(position,velocity); // To fix, it should be used only when trotting or with the arm
}

} // namespace
