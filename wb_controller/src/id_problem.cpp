#include <wb_controller/id_problem.h>
#include <OpenSoT/utils/Affine.h>
#include <wb_controller/utils.h>

using namespace OpenSoT;

#define CLASS_NAME "IDProblem"

IDProblem::IDProblem(ros::NodeHandle& nh, XBot::ModelInterface::Ptr model, std::vector<std::string> feet_names, std::string arm_tip_name):
    _model(model)
{

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
    _id.reset(new OpenSoT::utils::InverseDynamics(feet_names, *_model));

    //
    // Here we create all the tasks: the feet has to be created wrt the world frame
    //
    //   --------------------------
    for(unsigned int i=0; i<feet_names.size(); i++)
    {
        _feet[feet_names[i]].reset(new OpenSoT::tasks::acceleration::Cartesian(feet_names[i], *_model, feet_names[i],
                                                                               "world", _id->getJointsAccelerationAffine()));
        _feet[feet_names[i]]->setLambda(0.,0.);
        _feet[feet_names[i]]->setWeightIsDiagonalFlag(true);
    }
    //   --------------------------
    if(!arm_tip_name.empty())
    {
        ROS_INFO("Initialize ARM task");
        _arm.reset(new OpenSoT::tasks::acceleration::Cartesian(arm_tip_name, *_model, arm_tip_name,
                                                               "world", _id->getJointsAccelerationAffine()));
        _arm->setLambda(1.,1.);
        _arm->setWeightIsDiagonalFlag(true);

        idx_grfs_start_ = 23;
    }
    else {
        idx_grfs_start_ = 18;
    }

    //   --------------------------
    _waistRPY = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistRPY", *_model, "base_link",
                                                                            "world", _id->getJointsAccelerationAffine());
    _waistRPY->setLambda(1.,1.);
    _waistRPY->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _waistZ = std::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waistZ", *_model, "base_link",
                                                                        "world", _id->getJointsAccelerationAffine());
    _waistZ->setLambda(1.,1.);
    _waistZ->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _postural.reset(new OpenSoT::tasks::acceleration::Postural(*_model, _id->getJointsAccelerationAffine()));
    _postural->setLambda(1.,1.);
    _postural->setWeightIsDiagonalFlag(true);

    //   --------------------------
    _com.reset(new OpenSoT::tasks::acceleration::CoM(*_model, _id->getJointsAccelerationAffine()));
    _com->setLambda(0.,1.);
    _com->setWeightIsDiagonalFlag(true);

    //
    // Here we create the constraints & bounds
    //
    _dynamics_task.reset(new OpenSoT::tasks::acceleration::DynamicFeasibility("dynamics", *_model,
                                                                          _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), feet_names));

    _dynamics_con.reset(new OpenSoT::constraints::TaskToConstraint(_dynamics_task));


    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    _mu = 0.7;
    for(unsigned int i = 0; i < feet_names.size(); i++)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,_mu));
    _friction_cones.reset(new OpenSoT::constraints::force::FrictionCones(feet_names,_id->getContactsWrenchAffine(),*_model,mus));

    /// HERE WE SET SOME BOUNDS
    Eigen::VectorXd xmax = 500.*Eigen::VectorXd::Ones(_model->getJointNum());
    Eigen::VectorXd xmin = -xmax;

    _qddot_lims.reset(new OpenSoT::constraints::GenericConstraint(
                          "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));

    _x_force_lower_lim = -2000;
    _y_force_lower_lim = -2000;
    _z_force_lower_lim = default_z_lower_force;

    _wrench_upper_lims<<2000,2000,2000,Eigen::Vector3d::Zero();
    _wrench_lower_lims<<_x_force_lower_lim,_y_force_lower_lim,_z_force_lower_lim,Eigen::Vector3d::Zero();

    _wrenches_lims.reset(new OpenSoT::constraints::force::WrenchesLimits(
                             feet_names, _wrench_lower_lims, _wrench_upper_lims,_id->getContactsWrenchAffine()));

    for(unsigned int i = 0; i < _id->getContactsWrenchAffine().size(); ++i)
        _minfs.push_back(OpenSoT::tasks::MinimizeVariable::Ptr(new OpenSoT::tasks::MinimizeVariable("minf"+std::to_string(i), _id->getContactsWrenchAffine()[i])));

    std::list<unsigned int> id_XY = {0,1}; //xy
    std::list<unsigned int> id_Z = {2}; //z
    std::list<unsigned int> id_RPY = {3,4,5}; //r,p,y
    std::list<unsigned int> id_XYZ = {0,1,2};

    if(!arm_tip_name.empty()) // FIXME Use the operators....
    {
      //_stack = ((_feet[feet_names[0]]%idf + _feet[feet_names[1]]%idf + _feet[feet_names[2]]%idf + _feet[feet_names[3]]%idf)
      //        / (_waistRPY%idw_RPY)
      //        / (_arm)
      //        / (_postural)
      //        )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;

      //_stack /= ((6000*_feet[feet_names[0]]%idf + 6000*_feet[feet_names[1]]%idf + 6000*_feet[feet_names[2]]%idf + 6000*_feet[feet_names[3]]%idf
      //        + _waistRPY%idw_RPY + _postural + _com + _arm
      //        + 0.000001*_minfs[0] + 0.000001*_minfs[1] + 0.000001*_minfs[2] + 0.000001*_minfs[3])
      //        )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;

      // questo fa sballare l'arm
      //_stack = ((_feet[feet_names[0]]%idf + _feet[feet_names[1]]%idf + _feet[feet_names[2]]%idf + _feet[feet_names[3]]%idf)
      //        / (_waistRPY%idw_RPY + _arm + _postural + _com)
      //        )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;

      _stack = ((_feet[feet_names[0]]%id_XYZ + _feet[feet_names[1]]%id_XYZ + _feet[feet_names[2]]%id_XYZ + _feet[feet_names[3]]%id_XYZ)
              / (_com%id_XY)
              / (0.5*_waistRPY%id_RPY + 0.25*_waistZ%id_Z + _arm + 50.0*_com%id_Z)
              / (_postural)
              )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;

    }
    else
    {
        // NOTE: a stack with only one level is faster in terms of computational time and makes the robot move smoothly. The one with two levels is safer since the contacts would be at a higher priority but the
        // computation gets heavier. The second case can be divided in two more sub-cases depending on the RPY task of the base.
        // For the stack with the postural divided in stance and swing, checkout postural_swing_stance branch

#ifdef STACK_1
        // Stack with one level (gazebo rt factor 0.95):
        // we could use weights to emulate the case with two levels and save in this way some computational time or we could set the contacts as constraints.
        _stack /= ((6000*_feet[feet_names[0]]%idf + 6000*_feet[feet_names[1]]%idf + 6000*_feet[feet_names[2]]%idf + 6000*_feet[feet_names[3]]%idf
                + 1000.0*_waistRPY%idw_RPY + _postural + _com
                + 0.000001*_minfs[0] + 0.000001*_minfs[1] + 0.000001*_minfs[2] + 0.000001*_minfs[3])
                )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;
        ROS_INFO("------------------ PROBLEM STACK 1");
#endif

#ifdef STACK_2
        // Stack with two levels:
        // 1 - RPY at the first level (gazebo rt factor 0.75): this is useful to have a good tracking of the base orientation but it generates a conflict with the
        // postural e.g. the robot can not climb a slope if the base orientation is not adjusted to do so.
        _stack = ((1000.0*_feet[feet_names[0]]%idf + 1000.0*_feet[feet_names[1]]%idf + 1000.0*_feet[feet_names[2]]%idf + 1000.0*_feet[feet_names[3]]%idf + _waistRPY%idw_RPY)
                / (_postural)
                )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;
        ROS_INFO("------------------ PROBLEM STACK 2");
#endif

#ifdef  STACK_3
        // 2 - RPY at the second level (gazebo rt factor 0.45): this is generating a bad tracking for the base orientation which can be counteracted by adjusting the lambda gains or by adding weights between
        // postural and RPY.
        _stack = ((_feet[feet_names[0]]%idf + _feet[feet_names[1]]%idf + _feet[feet_names[2]]%idf + _feet[feet_names[3]]%idf)
                / (_waistRPY%idw_RPY + _postural)
                )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;
        ROS_INFO("------------------ PROBLEM STACK 3");
#endif

#ifdef STACK_4
        // Stack to perform the swing in the air, use freeze base to hold the robot in the air
        _stack /= (_postural) <<_qddot_lims<<_dynamics_con;
        ROS_INFO("------------------ PROBLEM STACK 4");
#endif

#ifdef STACK_5
        // Make the contacts hard constraints
        _stack = ((_feet[feet_names[0]]%id_XYZ + _feet[feet_names[1]]%id_XYZ + _feet[feet_names[2]]%id_XYZ + _feet[feet_names[3]]%id_XYZ)
                / (_waistRPY%id_RPY)
                / (_postural + _com)
                )<<_wrenches_lims<<_qddot_lims<<_dynamics_con<<_friction_cones;
        ROS_INFO("------------------ PROBLEM STACK 5");
#endif

    }

    _stack->update(Eigen::VectorXd(1));

    _solver.reset(new OpenSoT::solvers::iHQP(_stack->getStack(), _stack->getBounds(),1e6)); //, 1e6);
    //, OpenSoT::solvers::solver_back_ends::OSQP);
    //, OpenSoT::solvers::solver_back_ends::eiQuadProg);

    _x.setZero(_id->getSerializer()->getSize());

    _qddot.setZero(_model->getJointNum());
    _contact_wrenches.reserve(feet_names.size());

    // Add some ROS magic
    _tasks_ros["waistRPY"] = std::make_shared<CartesianWrapper>(nh,_waistRPY); // WAIST RPY
    _tasks_ros["waistZ"] = std::make_shared<CartesianWrapper>(nh,_waistZ); // WAIST Z
    for(unsigned int i=0; i<feet_names.size(); i++)
        _tasks_ros[feet_names[i]] = std::make_shared<CartesianWrapper>(nh,_feet[feet_names[i]]); // FEET
    _tasks_ros["postural"] = std::make_shared<PosturalWrapper>(nh,_postural); // POSTURAL
    _tasks_ros["com"] = std::make_shared<ComWrapper>(nh,_com); // CoM

    if(!arm_tip_name.empty())
        _tasks_ros["TCP"] = std::make_shared<CartesianWrapper>(nh,_arm); // ARM

    for (auto& tmp_map : _tasks_ros)
        tmp_map.second->dynamicReconfigureUpdate();

    // Set the callback for the dynamic reconfigure server
    ros::NodeHandle problem_nh("problem");
    server_.reset(new dynamic_reconfigure::Server<wb_controller::problemConfig>(problem_nh));
    server_->setCallback(boost::bind(&IDProblem::dynamicReconfigureCallback, this, _1, _2));

    dynamicReconfigureUpdate();
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
    case 2:
        setMinFsWeight(config.minFs_weight);
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
    default_config_.minFs_weight = _minfs[0]->getWeight()(0,0);

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

void IDProblem::setMinFsWeight(const double& weight)
{
    if(weight>=0.0)
    {
        for(unsigned int i=0;i<_minfs.size();i++)
        {
            _minfs[i]->setWeight(Eigen::Matrix6d::Identity()*weight); // FIXME No-RT safe
            ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set "<<_minfs[i]->getTaskID()<<" weight to: "<<weight);
        }
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Weight has to be positive!");
}


void IDProblem::reset()
{
    _postural->reset();
    _waistRPY->reset();
    _waistZ->reset();
    _com->reset();
    if(_arm)
        _arm->reset();
    for (auto& tmp_map : _feet)
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
    _stack->update(Eigen::VectorXd(1));
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
    bool a = _solver->solve(_x);
    if(!a)
        return false;
    a = _id->computedTorque(_x, tau, _qddot, _contact_wrenches);

    return a;
}

void IDProblem::getGroundReactionForces(Eigen::VectorXd& grfs)
{
    //std::cout << "   FORCES GRF *********** " << std::endl;
    grfs = _x.segment(idx_grfs_start_,24);

    //std::cout << "   ACCELLERATIONS FB *********** " << std::endl;
    //std::cout << _x.segment(0,6).transpose() << std::endl;
}
