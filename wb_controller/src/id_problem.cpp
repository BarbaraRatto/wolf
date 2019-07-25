#include <wb_controller/id_problem.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <wb_controller/utils.h>

using namespace OpenSoT;

IDProblem::IDProblem(ros::NodeHandle& nh, XBot::ModelInterface::Ptr model, const double dT, std::vector<std::string> feet_names, std::string arm_tip_name):
    _model(model)
{
    //
    // With links_in_contact we define which links are in contact with the environment
    //

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
        _feet[feet_names[i]]->setLambda(6.,12.); // 60. 6.
        _feet[feet_names[i]]->setWeightIsDiagonalFlag(true);
    }
     //   --------------------------
    if(!arm_tip_name.empty())
    {
        ROS_INFO("Initialize ARM task");
        _arm.reset(new OpenSoT::tasks::acceleration::Cartesian(arm_tip_name, *_model, arm_tip_name,
                                                                         "base_link", _id->getJointsAccelerationAffine()));
        _arm->setLambda(100.);
        _arm->setWeightIsDiagonalFlag(true);

        idx_grfs_start_ = 23;
    }
    else {
        idx_grfs_start_ = 18;
    }

    //   --------------------------
    _waist = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waist", *_model, "base_link",
                                                                         "world", _id->getJointsAccelerationAffine());
    _waist->setLambda(100.);
    _waist->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _postural.reset(new OpenSoT::tasks::acceleration::Postural(*_model, _id->getJointsAccelerationAffine()));
    _postural->setLambda(100.);
    _postural->setWeightIsDiagonalFlag(true);
    //   --------------------------
    _com.reset(new OpenSoT::tasks::acceleration::CoM(*_model, _id->getJointsAccelerationAffine()));
    _com->setLambda(0.);
    _com->setWeightIsDiagonalFlag(true);

    //
    // Here we create the constraints & bounds
    //
    _dynamics.reset(new OpenSoT::constraints::acceleration::DynamicFeasibility("dynamics", *_model,
                                                                               _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), feet_names));
    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    for(unsigned int i = 0; i < feet_names.size(); i++)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,0.5));
    //_friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCone>(_id->getContactsWrenchAffine(),*_model,mus);
    _friction_cones.reset(new OpenSoT::constraints::force::FrictionCones(feet_names,_id->getContactsWrenchAffine(),*_model,mus));

    /// HERE WE SET SOME BOUNDS
    Eigen::VectorXd xmax = 100.*Eigen::VectorXd::Ones(_model->getJointNum());
    Eigen::VectorXd xmin = -xmax;

    _qddot_lims.reset(new OpenSoT::constraints::GenericConstraint(
                          "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));

    //_qddot_lims = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
    //            "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    Eigen::Vector6d wrench_upper_lims; wrench_upper_lims<<1000,1000,1000,Eigen::Vector3d::Zero();
    Eigen::Vector6d wrench_lower_lims; wrench_lower_lims<<-1000, -1000, 20.0 ,Eigen::Vector3d::Zero();
    _wrenches_lims.reset(new OpenSoT::constraints::force::WrenchesLimits(
                             feet_names, wrench_lower_lims, wrench_upper_lims,_id->getContactsWrenchAffine()));

    std::vector<OpenSoT::tasks::MinimizeVariable::Ptr> minfs;
    for(unsigned int i = 0; i < _id->getContactsWrenchAffine().size(); ++i)
        minfs.push_back(OpenSoT::tasks::MinimizeVariable::Ptr(new OpenSoT::tasks::MinimizeVariable("minf"+std::to_string(i), _id->getContactsWrenchAffine()[i])));

    std::list<unsigned int> idw = {2,3,4,5}; //z,r,p,y
    //std::list<unsigned int> idw = {3,4,5}; //r,p,y
    std::list<unsigned int> idf = {0,1,2};

    if(!arm_tip_name.empty()) // FIXME Use the operators....
    {
        _id_problem = ((_feet[feet_names[0]]%idf + _feet[feet_names[1]]%idf + _feet[feet_names[2]]%idf + _feet[feet_names[3]]%idf + _waist%idw)
                /(_arm)/(_postural)
                )<<_wrenches_lims<<_qddot_lims<<_dynamics<<_friction_cones;
    }
    else
    {
        /*_id_problem = ((_feet[feet_names[0]]%idf + _feet[feet_names[1]]%idf + _feet[feet_names[2]]%idf + _feet[feet_names[3]]%idf + _waist%idw)
                /(_postural)
                )<<_wrenches_lims<<_qddot_lims<<_dynamics<<_friction_cones;*/
        //_id_problem /= (_postural)<<_wrenches_lims<<_qddot_lims<<_dynamics<<_friction_cones;

        _id_problem = ((_feet[feet_names[0]]%idf + _feet[feet_names[1]]%idf + _feet[feet_names[2]]%idf + _feet[feet_names[3]]%idf + _waist%idw)
                /(_postural)
                )<<_wrenches_lims<<_qddot_lims<<_dynamics<<_friction_cones;
    }
    // /(_com%idc + _waist%idw))<<_qddot_lims<<_wrenches_lims<<_dynamics<<_friction_cones;

    _id_problem->update(Eigen::VectorXd(1));

    _solver.reset(new OpenSoT::solvers::iHQP(_id_problem->getStack(), _id_problem->getBounds(),1e6)); //, 1e6);
    //, OpenSoT::solvers::solver_back_ends::OSQP);
    //, OpenSoT::solvers::solver_back_ends::eiQuadProg);

    _x.setZero(_id->getSerializer()->getSize());

    _qddot.setZero(_model->getJointNum());


    // Add some ROS magic
    _tasks_ros["waist"] = std::make_shared<CartesianWrapper>(nh,_waist); // WAIST
    for(unsigned int i=0; i<feet_names.size(); i++)
        _tasks_ros[feet_names[i]] = std::make_shared<CartesianWrapper>(nh,_feet[feet_names[i]]); // FEET

     _tasks_ros["com"] = std::make_shared<ComWrapper>(nh,_com); // COM
     _tasks_ros["postural"] = std::make_shared<PosturalWrapper>(nh,_postural); // POSTURAL

     if(!arm_tip_name.empty())
         _tasks_ros["TCP"] = std::make_shared<CartesianWrapper>(nh,_arm); // ARM

     for (auto& tmp_map : _tasks_ros)
         tmp_map.second->dynamicReconfigureUpdate();
}

void IDProblem::reset()
{
    _postural->reset();
    _waist->reset();
    _com->resetReference();
    if(_arm)
        _arm->reset();
    for (auto& tmp_map : _feet)
        tmp_map.second->reset();
}

void IDProblem::update()
{
    _id_problem->update(Eigen::VectorXd(1));
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
    a = _id->computedTorque(_x, tau, _qddot);

    return a;
}

void IDProblem::getGroundReactionForces(Eigen::VectorXd& grfs)
{
    //std::cout << "   FORZE GRF *********** " << std::endl;
    grfs = _x.segment(idx_grfs_start_,24);

    //std::cout << "   ACC FB *********** " << std::endl;
    //std::cout << _x.segment(0,6).transpose() << std::endl;
}

void IDProblem::log(XBot::MatLogger::Ptr& logger)
{
    _id->log(logger);
    _id_problem->log(logger);
    _solver->log(logger);
}

IDProblem::~IDProblem()
{

}


