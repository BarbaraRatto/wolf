#include <dls_controller/IDProblem.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/MinimizeVariable.h>

using namespace OpenSoT;

IDProblem::IDProblem(ros::NodeHandle& nh, XBot::ModelInterface::Ptr model, const double dT, std::vector<std::string>& contact_links):
    _model(model)
{
    //
    // With links_in_contact we define which links are in contact with the environment
    //

    //
    //  This utility internally creates the right variables which later we will use to
    //  create all the tasks and constraints
    //
    _id.reset(new OpenSoT::utils::InverseDynamics(contact_links, *_model));

    //
    // Here we create all the tasks: the feet has to be created wrt the world frame
    //
    for(unsigned int i=0; i<contact_links.size(); i++)
    {
        _feet[contact_links[i]].reset(new OpenSoT::tasks::acceleration::Cartesian(contact_links[i], *_model, contact_links[i],
                                                                                  "world", _id->getJointsAccelerationAffine()));
        _feet[contact_links[i]]->setLambda(0.,10.);
        _feet[contact_links[i]]->setWeightIsDiagonalFlag(true);
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
                                                                               _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), contact_links));
    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    for(unsigned int i = 0; i < contact_links.size(); ++i)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,0.5));
    //_friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCone>(_id->getContactsWrenchAffine(),*_model,mus);
    _friction_cones.reset(new OpenSoT::constraints::force::FrictionCones(contact_links,_id->getContactsWrenchAffine(),*_model,mus));

    /// HERE WE SET SOME BOUNDS
    Eigen::VectorXd xmax = 100.*Eigen::VectorXd::Ones(_model->getJointNum());
    Eigen::VectorXd xmin = -xmax;

    _qddot_lims.reset(new OpenSoT::constraints::GenericConstraint(
                          "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));

    //_qddot_lims = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
    //            "acc_lims", _id->getJointsAccelerationAffine(), xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    Eigen::Vector6d wrench_upper_lims; wrench_upper_lims<<1000,1000,1000,Eigen::Vector3d::Zero();
    Eigen::Vector6d wrench_lower_lims; wrench_lower_lims<<-1000, -1000, 10.0 ,Eigen::Vector3d::Zero();
    _wrenches_lims.reset(new OpenSoT::constraints::force::WrenchesLimits(
                             contact_links, wrench_lower_lims, wrench_upper_lims,_id->getContactsWrenchAffine()));

    std::vector<OpenSoT::tasks::MinimizeVariable::Ptr> minfs;
    for(unsigned int i = 0; i < _id->getContactsWrenchAffine().size(); ++i)
        minfs.push_back(OpenSoT::tasks::MinimizeVariable::Ptr(new OpenSoT::tasks::MinimizeVariable("minf"+std::to_string(i), _id->getContactsWrenchAffine()[i])));
    // Notice that we just control the orientation of the waist and the z
    std::list<unsigned int> idw = {2,3,4,5};
    std::list<unsigned int> idf = {0,1,2};

    _id_problem = ((_feet[contact_links[0]]%idf + _feet[contact_links[1]]%idf + _feet[contact_links[2]]%idf + _feet[contact_links[3]]%idf + _waist%idw)
            /(_postural)
            )<<_wrenches_lims<<_qddot_lims<<_dynamics<<_friction_cones;//<<_qddot_lims<<_wrenches_lims
    // /(_com%idc + _waist%idw))<<_qddot_lims<<_wrenches_lims<<_dynamics<<_friction_cones;

    _id_problem->update(Eigen::VectorXd(1));

    _solver.reset(new OpenSoT::solvers::iHQP(_id_problem->getStack(), _id_problem->getBounds(),1e6)); //, 1e6);
    //, OpenSoT::solvers::solver_back_ends::OSQP);
    //, OpenSoT::solvers::solver_back_ends::eiQuadProg);

    _x.setZero(_id->getSerializer()->getSize());

    _qddot.setZero(_model->getJointNum());


    _tasks_ros.push_back(TaskRosWrapper::Ptr(new TaskRosWrapper(nh,_waist))); // WAIST
    for(unsigned int i=0; i<contact_links.size(); i++)
        _tasks_ros.push_back(TaskRosWrapper::Ptr(new TaskRosWrapper(nh,_feet[contact_links[i]]))); //FEET

    // Add some ROS magic
    /*_tasks_ros.push_back(TaskRosWrapper::Ptr(new TaskRosWrapper(nh,_waist))); // WAIST
    _tasks_ros.push_back(TaskRosWrapper::Ptr(new TaskRosWrapper(nh,_waist))); // WAIST
    _tasks_ros.push_back(boost::make_shared<TaskRosWrapper>(nh,_postural)); // POSTURAL*/


}

void IDProblem::update()
{
    _id_problem->update(Eigen::VectorXd(1));
}

void IDProblem::publish(const ros::Time& time)
{
    for(unsigned int i =0;i< _tasks_ros.size(); i++)
        _tasks_ros[i]->publish(time);
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
    grfs = _x.segment(18,24);

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


TaskRosWrapper::TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::Cartesian::Ptr task)
{
    assert(task);
    task_ = task;

    rt_pub_.reset(new realtime_tools::RealtimePublisher<dls_controller::Task>(nh, task->getTaskID(), 4));
}

void TaskRosWrapper::publish(const ros::Time& time)
{
    if(rt_pub_->trylock())
    {
        rt_pub_->msg_.header.frame_id = task_->getBaseLink();
        rt_pub_->msg_.header.stamp = time;

        task_->getActualPose(tmp_affine3d_);
        task_->getActualTwist(tmp_vector6d_);

        tmp_quaterniond_ = tmp_affine3d_.linear();

        // Pose - Translation
        rt_pub_->msg_.pose.position.x = tmp_affine3d_.translation().x();
        rt_pub_->msg_.pose.position.y = tmp_affine3d_.translation().y();
        rt_pub_->msg_.pose.position.z = tmp_affine3d_.translation().z();
        // Pose - Linear
        rt_pub_->msg_.pose.orientation.x = tmp_quaterniond_.x();
        rt_pub_->msg_.pose.orientation.y = tmp_quaterniond_.y();
        rt_pub_->msg_.pose.orientation.z = tmp_quaterniond_.z();
        rt_pub_->msg_.pose.orientation.w = tmp_quaterniond_.w();
        // Twist
        rt_pub_->msg_.twist.linear.x  = tmp_vector6d_(0);
        rt_pub_->msg_.twist.linear.y  = tmp_vector6d_(1);
        rt_pub_->msg_.twist.linear.z  = tmp_vector6d_(2);
        rt_pub_->msg_.twist.angular.x = tmp_vector6d_(3);
        rt_pub_->msg_.twist.angular.y = tmp_vector6d_(4);
        rt_pub_->msg_.twist.angular.z = tmp_vector6d_(5);

        rt_pub_->unlockAndPublish();

    }

}

