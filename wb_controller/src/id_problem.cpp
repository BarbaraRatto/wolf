#include <wb_controller/id_problem.h>
#include <OpenSoT/utils/Affine.h>
#include <wb_controller/utils.h>

using namespace OpenSoT;

namespace wb_controller {

#define CLASS_NAME "IDProblem"

IDProblem::IDProblem(ros::NodeHandle& nh, QuadrupedRobot::Ptr model):
    _model(model),
    _current_stack(stacks_t::NONE)
{

    foot_names_    = _model->getFootNames();
    arm_names_     = _model->getArmNames();
    contact_names_ = _model->getContactNames();

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
    _id = std::make_shared<OpenSoT::utils::InverseDynamics>(foot_names_, *_model->getXBotModel());

    //
    // Here we create all the tasks
    //   --------------------------
    for(unsigned int i=0; i<foot_names_.size(); i++)
    {
        ROS_INFO("Initialize FOOT tasks");
        _feet[foot_names_[i]] = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>(foot_names_[i], *_model->getXBotModel(), foot_names_[i],
                                                                               "world", _id->getJointsAccelerationAffine());
        _feet[foot_names_[i]]->setLambda(0.,0.);
        _feet[foot_names_[i]]->setWeightIsDiagonalFlag(true);
    }
    //   --------------------------
    for(unsigned int i=0; i<arm_names_.size(); i++)
    {
        ROS_INFO("Initialize ARM tasks");
        _arms[arm_names_[i]] = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>(arm_names_[i], *_model->getXBotModel(), arm_names_[i],
                                                               "base_link", _id->getJointsAccelerationAffine());
        _arms[arm_names_[i]]->setLambda(1,1);
        _arms[arm_names_[i]]->setWeightIsDiagonalFlag(true);

    }
   //     idx_grfs_start_ = 23; // FIXME
   // }
   // else {
   //     idx_grfs_start_ = 18; // FIXME
   // }
    //   --------------------------
    _angular_momentum = std::make_shared<OpenSoT::tasks::acceleration::AngularMomentum>(*_model->getXBotModel(),_id->getJointsAccelerationAffine());
    _angular_momentum->setLambda(0.);
    _angular_momentum->setWeightIsDiagonalFlag(true);
    _angular_momentum->setReference(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    _angular_momentum->setMomentumGain(Eigen::Matrix3d::Identity());
    //   --------------------------
    _waistRPY = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistRPY", *_model->getXBotModel(), "base_link",
                                                                          "world", _id->getJointsAccelerationAffine());
    _waistRPY->setLambda(1.,1.);
    _waistRPY->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _waistZ = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistZ", *_model->getXBotModel(), "base_link",
                                                                        "world", _id->getJointsAccelerationAffine());
    _waistZ->setLambda(1.,1.);
    _waistZ->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _postural = std::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model->getXBotModel(), _id->getJointsAccelerationAffine());
    _postural->setLambda(1.,1.);
    _postural->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _com = std::make_shared<OpenSoT::tasks::acceleration::CoM>(*_model->getXBotModel(), _id->getJointsAccelerationAffine());
    _com->setLambda(0.,100.);
    _com->setWeightIsDiagonalFlag(true);

    //
    // Here we create the constraints & bounds
    //
    _dynamics_task = std::make_shared<OpenSoT::tasks::acceleration::DynamicFeasibility>("dynamics", *_model->getXBotModel(),
                                                                          _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), contact_names_);

    _dynamics_con = std::make_shared<OpenSoT::constraints::TaskToConstraint>(_dynamics_task);


    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    _mu = 0.7;
    for(unsigned int i = 0; i < foot_names_.size(); i++)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,_mu));
    _friction_cones = std::make_shared<OpenSoT::constraints::force::FrictionCones>(foot_names_,_id->getContactsWrenchAffine(),*_model,mus);

    /// HERE WE SET SOME BOUNDS
    Eigen::VectorXd xmax = 500.*Eigen::VectorXd::Ones(_model->getXBotModel()->getJointNum());
    Eigen::VectorXd xmin = -xmax;

    _qddot_lims = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                          "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    _x_force_lower_lim = -2000;
    _y_force_lower_lim = -2000;
    _z_force_lower_lim = default_z_lower_force;

    _wrench_upper_lims<<2000,2000,2000,Eigen::Vector3d::Zero();
    _wrench_lower_lims<<_x_force_lower_lim,_y_force_lower_lim,_z_force_lower_lim,Eigen::Vector3d::Zero();

    _wrenches_lims = std::make_shared<OpenSoT::constraints::force::WrenchesLimits>(
                             contact_names_, _wrench_lower_lims, _wrench_upper_lims,_id->getContactsWrenchAffine());

    std::list<unsigned int> id_XY  = {0,1};   //xy
    std::list<unsigned int> id_Z   = {2};     //z
    std::list<unsigned int> id_RPY = {3,4,5}; //r,p,y
    std::list<unsigned int> id_XYZ = {0,1,2}; //xyz

    OpenSoT::tasks::Aggregated::Ptr feet_aggregated;
    for(unsigned int i=0;i<foot_names_.size()-1;i++)
      feet_aggregated = _feet[foot_names_[i]]%id_XYZ + _feet[foot_names_[i+1]]%id_XYZ;

    if(arm_names_.size()>0)
    {

      OpenSoT::tasks::Aggregated::Ptr arm_aggregated;
      for(unsigned int i=0;i<arm_names_.size()-1;i++)
        arm_aggregated = _arms[arm_names_[i]] + _arms[arm_names_[i+1]];

      // Stack manipolazione con arm wrt world
      _stacks[MANIPULATION] = ( (feet_aggregated)
                              / (_com%id_XY)
                              / (0.5*_waistRPY%id_RPY + 0.25*_waistZ%id_Z + arm_aggregated + 50.0*_com%id_Z + _angular_momentum)
                              / (_postural)
                              )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;

      // Stack base_link camminata, ok ad alte frequenze
      _stacks[WALKING] = ( (feet_aggregated)
                         / (_waistRPY%id_RPY)
                         / (arm_aggregated)
                         / (_postural + _com + _angular_momentum)
                         )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;
    }
    else
    {

       _stacks[WALKING] = ( (feet_aggregated)
                          / (_waistRPY%id_RPY)
                          / (_postural + _com + _angular_momentum) // Dunno if it really change anything with the angular momentum and the com task
                          )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;

    }

    for (auto& tmp_map : _stacks)
        tmp_map.second->update(Eigen::VectorXd(1));

    _x.setZero(_id->getSerializer()->getSize());

    _qddot.setZero(_model->getXBotModel()->getJointNum());
    _contact_wrenches.reserve(contact_names_.size());

    // Add some ROS magic
    _tasks_ros["waistRPY"] = std::make_shared<CartesianWrapper>(nh,_waistRPY); // WAIST RPY
    _tasks_ros["waistZ"] = std::make_shared<CartesianWrapper>(nh,_waistZ); // WAIST Z
    _tasks_ros["postural"] = std::make_shared<PosturalWrapper>(nh,_postural); // POSTURAL
    _tasks_ros["com"] = std::make_shared<ComWrapper>(nh,_com); // CoM
    for(unsigned int i=0; i<foot_names_.size(); i++)
        _tasks_ros[foot_names_[i]] = std::make_shared<CartesianWrapper>(nh,_feet[foot_names_[i]]); // FEET
    for(unsigned int i=0; i<arm_names_.size(); i++)
        _tasks_ros[arm_names_[i]] = std::make_shared<CartesianWrapper>(nh,_arms[arm_names_[i]]); // ARMS

    for (auto& tmp_map : _tasks_ros)
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
    default_config_.mu = _mu;
    default_config_.x_force_lower_lim = _x_force_lower_lim;
    default_config_.y_force_lower_lim = _y_force_lower_lim;
    default_config_.z_force_lower_lim = _z_force_lower_lim;

    if(server_)
        server_->updateConfig(default_config_);
}

void IDProblem::setFrictionConesMu(const double& mu)
{
    if(mu>=0.0 && mu<=1.0)
    {
        _mu = mu;
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set mu to: "<<mu);
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Mu has to be between 0 and 1!");
}

void IDProblem::setLowerForceBound(const double& x_force,const double& y_force,const double& z_force)
{
    _x_force_lower_lim = x_force;
    _y_force_lower_lim = y_force;
    _z_force_lower_lim = z_force;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set x force lower lim to: "<<x_force);
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set y force lower lim to: "<<y_force);
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set z force lower lim to: "<<z_force);
}

void IDProblem::setLowerForceBoundX(const double& force)
{
    _x_force_lower_lim = force;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set x force lower lim to: "<<force);
}

void IDProblem::setLowerForceBoundY(const double& force)
{
    _y_force_lower_lim = force;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set y force lower lim to: "<<force);
}

void IDProblem::setLowerForceBoundZ(const double& force)
{
    _z_force_lower_lim = force;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set z force lower lim to: "<<force);
}

void IDProblem::selectStack(const stacks_t& stack)
{

  if(_current_stack != stack)
  {
    _current_stack = stack;

    _solver_lock.lock();

    switch (_current_stack)
    {
      case stacks_t::WALKING:
        for (auto& tmp_map : _arms)
        {
          tmp_map.second->setBaseLink("base_link");
          tmp_map.second->update(Eigen::VectorXd(1));
        }
        _tasks_ros["TCP"]->reset();
        ROS_INFO_NAMED(CLASS_NAME,"STACK WALKING SELECTED");
        break;
      case stacks_t::MANIPULATION:
        for (auto& tmp_map : _arms)
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
      _tasks_ros[arm_names_[i]]->reset();

    //std::cout << "_solver = std::make_unique<OpenSoT::solvers::iHQP>" << std::endl;
    if(_solver.get()!=nullptr)
      _solver.release();
    _solver = std::make_unique<OpenSoT::solvers::iHQP>(_stacks[_current_stack]->getStack(), _stacks[_current_stack]->getBounds(),1e6); //, 1e6);
    // , OpenSoT::solvers::solver_back_ends::OSQP);
    // , OpenSoT::solvers::solver_back_ends::eiQuadProg);
    _solver_lock.unlock();
  }
}

void IDProblem::switchStack()
{
  if(_current_stack == stacks_t::WALKING)
    selectStack(stacks_t::MANIPULATION);
  else
   selectStack(stacks_t::WALKING);
}

unsigned int IDProblem::getCurrentStack()
{
  return _current_stack;
}

void IDProblem::reset()
{
  for (auto& tmp_map : _tasks_ros)
      tmp_map.second->reset();
}

void IDProblem::update()
{
    // Update the mu and the wrench limits
    _wrench_lower_lims(0) = _x_force_lower_lim;
    _wrench_lower_lims(1) = _y_force_lower_lim;
    _wrench_lower_lims(2) = _z_force_lower_lim;
    for (auto& tmp_map : _feet)
    {
        _friction_cones->getFrictionCone(tmp_map.first)->setMu(_mu);
        if(!_wrenches_lims->getWrenchLimits(tmp_map.first)->isReleased())
            _wrenches_lims->getWrenchLimits(tmp_map.first)->setWrenchLimits(_wrench_lower_lims,_wrench_upper_lims);
    }
    // Update the problem
    _stacks[_current_stack]->update(Eigen::VectorXd(1));
}

void IDProblem::updateReference(const std::string& task_name)
{
    if(_tasks_ros.count(task_name))
        _tasks_ros[task_name]->updateReference();
}

void IDProblem::publish(const ros::Time& time)
{
    for (auto& tmp_map : _tasks_ros)
        tmp_map.second->publish(time);
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
    update();

    bool res_solv = false;
    bool res_id = false;
    if (_solver_lock.try_lock())
    {
      res_solv = _solver->solve(_x);
      if(res_solv)
        res_id = _id->computedTorque(_x, tau, _qddot, _contact_wrenches);
      _solver_lock.unlock();
    }
    return (res_solv && res_id);
}

void IDProblem::getGroundReactionForces(Eigen::VectorXd& grfs)
{
    //std::cout << "   FORCES GRF *********** " << std::endl;
    grfs = _x.segment(idx_grfs_start_,24);

    //std::cout << "   ACCELLERATIONS FB *********** " << std::endl;
    //std::cout << _x.segment(0,6).transpose() << std::endl;
}

} // namespace
