#include <dls_controller/IDProblem.h>
#include <OpenSoT/utils/Affine.h>

using namespace OpenSoT;

IDProblem::IDProblem(XBot::ModelInterface::Ptr model, const double dT, std::vector<std::string>& contact_links):
    _model(model)
{
    //
    // With links_in_contact we define which links are in contact with the environment
    //

    //
    //  This utility internally creates the right variables which later we will use to
    //  create all the tasks and constraints
    //
    _id = boost::make_shared<OpenSoT::utils::InverseDynamics>(contact_links, *_model);

    //
    // Here we create all the tasks: the feet has to be created wrt the world frame
    //
    for(unsigned int i=0; i<contact_links.size(); i++)
    {
        _feet[contact_links[i]] = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>(contact_links[i], *_model, contact_links[i],
                                                                               "world", _id->getJointsAccelerationAffine());
        _feet[contact_links[i]]->setLambda(2000.);
    }
    //   --------------------------
    _waist = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waist", *_model, "base_link",
                                                                        "world", _id->getJointsAccelerationAffine());
    _waist->setLambda(400.);
    //   --------------------------
    _postural = boost::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model, _id->getJointsAccelerationAffine());
    _postural->setLambda(400.);
    //   --------------------------
    _com = boost::make_shared<OpenSoT::tasks::acceleration::CoM>(*_model, _id->getJointsAccelerationAffine());
    _com->setLambda(800.);

    //
    // Here we create the constraints & bounds
    //
    _dynamics = boost::make_shared<OpenSoT::constraints::acceleration::DynamicFeasibility>("dynamics", *_model,
                                                                                           _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), contact_links);
    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    for(unsigned int i = 0; i < contact_links.size(); ++i)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,0.5));
    //_friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCone>(_id->getContactsWrenchAffine(),*_model,mus);
    _friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCones>(contact_links,_id->getContactsWrenchAffine(),*_model,mus);


    /// HERE WE SET SOME BOUNDS
    Eigen::VectorXd xmax = 1000.*Eigen::VectorXd::Ones(_model->getJointNum());
    Eigen::VectorXd xmin = -xmax;

    _qddot_lims = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    Eigen::Vector6d wrench_upper_lims; wrench_upper_lims<<1000,1000,1000,Eigen::Vector3d::Zero();
    Eigen::Vector6d wrench_lower_lims; wrench_lower_lims<<-1000, -1000, 0.0 ,Eigen::Vector3d::Zero();
    _wrenches_lims = boost::make_shared<OpenSoT::constraints::force::WrenchesLimits>(
                contact_links, wrench_lower_lims, wrench_upper_lims,_id->getContactsWrenchAffine());

    // Notice that we just control the orientation of the waist
    std::list<unsigned int> idw = {2,3,4,5};
    std::list<unsigned int> idc = {2};
    std::list<unsigned int> idf = {0,1,2};

    _id_problem = ((_feet[contact_links[0]]%idf + _feet[contact_links[1]]%idf + _feet[contact_links[2]]%idf + _feet[contact_links[3]]%idf)
            /(_waist%idw)/_postural)<<_qddot_lims<<_wrenches_lims<<_dynamics<<_friction_cones;

    _id_problem->update(Eigen::VectorXd(1));

    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_id_problem->getStack(), _id_problem->getBounds(),1e6); //, 1e6);
                                                         //, OpenSoT::solvers::solver_back_ends::OSQP);
                                                         //, OpenSoT::solvers::solver_back_ends::eiQuadProg);

    _x.setZero(_id->getSerializer()->getSize());

    _qddot.setZero(_model->getJointNum());
}

void IDProblem::update()
{
    _id_problem->update(Eigen::VectorXd(1));
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
    bool a = _solver->solve(_x);
    if(!a)
        return false;
    a = _id->computedTorque(_x, tau, _qddot);

    //std::cout << "   FORZE GRF *********** " << std::endl;
    //std::cout << _x.segment(18,24).transpose() << std::endl;

    //std::cout << "   ACC FB *********** " << std::endl;
    //std::cout << _x.segment(0,6).transpose() << std::endl;

    return a;
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

