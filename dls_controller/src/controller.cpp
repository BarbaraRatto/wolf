/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2018
 * @brief DLS Controller.
 *
 * This file contains the constructor, destructor, init, stopping and other facilities for the
 * DLS Controller.
 * @see git@gitlab.advr.iit.it:dls-lab/dls_core.git
 */

#include <dls_controller/controller.h>

using namespace XBot;
using namespace Cartesian;

namespace dls_controller {

#define FLOATING_BASE_DOFS 6
#define DT 0.001 // FIXME
#define CONTROLLER_NAME "dls_controller"

Controller::Controller()
    :solver_started_(false)
    ,pid_active_(true)
    ,tracking_active_(false)
    ,stopping_(false)
{
}

Controller::~Controller()
{
    if(ci_joint_states_rt_pub_)
        delete ci_joint_states_rt_pub_;
    if(state_estimation_rt_pub_)
        delete state_estimation_rt_pub_;
    if(tasks_actual_pose_rt_pub_)
      delete tasks_actual_pose_rt_pub_;
    if(server_)
      delete server_;
}

bool Controller::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // getting the names of the joints from the ROS parameter server
    ROS_DEBUG("Initialize DLS Controller");

    assert(robot_hw);

    hardware_interface::JointCommandAdvInterface* jt_hw = robot_hw->get<hardware_interface::JointCommandAdvInterface>();
    hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
    hardware_interface::GroundTruthInterface* gt_hw = robot_hw->get<hardware_interface::GroundTruthInterface>();
    hardware_interface::ContactSwitchSensorInterface* cont_hw = robot_hw->get<hardware_interface::ContactSwitchSensorInterface>();

    if(!jt_hw)
    {
        ROS_ERROR("hardware_interface::JointCommandAdvInterface not found");
        return false;
    }
    if(!imu_hw)
    {
        ROS_ERROR("hardware_interface::ImuSensorInterface not found");
        return false;
    }
    if(!gt_hw)
    {
        ROS_ERROR("hardware_interface::GroundTruthInterface not found");
        return false;
    }
    if(!cont_hw)
    {
        ROS_ERROR("hardware_interface::ContactSwitchSensorInterface not found");
        return false;
    }
    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    if (!controller_nh.getParam("imu_sensors", imu_names_))
    {
        ROS_ERROR("No imu_sensors given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    if (!controller_nh.getParam("state_estimators", state_estimator_names_))
    {
        ROS_ERROR("No state_estimators given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    if (!controller_nh.getParam("contact_sensors", contact_sensor_names_))
    {
        ROS_ERROR("No contact_sensors given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    // Setting up handles:
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        // Getting joint state handle
        try
        {
            ROS_DEBUG_STREAM("Found joint: "<<joint_names_[i]);
            joint_states_.push_back(jt_hw->getHandle(joint_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading joint_states_");
            return false;
        }
    }
    assert(joint_states_.size()>0);

    for (unsigned int i = 0; i < imu_names_.size(); i++)
    {
        try
        {
            ROS_DEBUG_STREAM("Found imu sensor: "<<imu_names_[i]);
            imu_sensors_.push_back(imu_hw->getHandle(imu_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading imu_sensors_");
            return false;
        }
    }
    assert(imu_sensors_.size()>0);

    for (unsigned int i = 0; i < state_estimator_names_.size(); i++)
    {
        try
        {
            ROS_DEBUG_STREAM("Found state estimator: "<< state_estimator_names_[i]);
            state_estimators_.push_back(gt_hw->getHandle(state_estimator_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading state_estimators_");
            return false;
        }
    }
    for (unsigned int i = 0; i < contact_sensor_names_.size(); i++)
    {
        try
        {
            ROS_DEBUG_STREAM("Found contact sensor: "<<contact_sensor_names_[i]);
            contact_sensors_.push_back(cont_hw->getHandle(contact_sensor_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading contact_sensors_");
            return false;
        }
    }
    assert(contact_sensors_.size()>0);

    des_joint_p_gain_.resize(joint_states_.size());
    des_joint_i_gain_.resize(joint_states_.size());
    des_joint_d_gain_.resize(joint_states_.size());
    joint_p_gain_.resize(joint_states_.size());
    joint_i_gain_.resize(joint_states_.size());
    joint_d_gain_.resize(joint_states_.size());
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // Getting PID gains
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", joint_p_gain_[i]))
        {
            ROS_ERROR("No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", joint_i_gain_[i]))
        {
            ROS_ERROR("No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", joint_d_gain_[i]))
        {
            ROS_ERROR("No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        // Check if the values are positive
        if(joint_p_gain_[i]<0.0 || joint_i_gain_[i]<0.0 || joint_d_gain_[i]<0.0)
        {
            ROS_ERROR("PID gains must be positive!");
            return false;
        }
        ROS_DEBUG("P value for joint %i is: %d",i,joint_p_gain_[i]);
        ROS_DEBUG("I value for joint %i is: %d",i,joint_i_gain_[i]);
        ROS_DEBUG("D value for joint %i is: %d",i,joint_d_gain_[i]);

        // Set the gain value when the error is 0 and the gain value when the error reach x [m]
        //double x = 0.1;
        //adaptive_joint_p_gain_.push_back(new AdaptiveGain(des_joint_p_gain_[i],des_joint_p_gain_[i]/2.0,x));
    }

    // Create the ModelInterface from XBot
    XBot::ConfigOptions opt;
    std::string urdf, srdf, problem;

    if(!root_nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
    {
        ROS_ERROR_STREAM_NAMED(CONTROLLER_NAME,"No robot_description given in namespace /");
        return false;
    }
    if(!root_nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
    {
        ROS_ERROR_STREAM_NAMED(CONTROLLER_NAME,"No robot_semantic_description given in namespace /");
        return false;
    }
    if(!opt.set_urdf(urdf))
    {
       ROS_ERROR("Unable to load urdf");
       return false;
    }
    if(!opt.set_srdf(srdf))
    {
       ROS_ERROR("Unable to load srdf");
       return false;
    }
    if(!opt.generate_jidmap())
    {
        ROS_ERROR("Unable to load jidmap");
        return false;
    }
    opt.set_parameter("is_model_floating_base", true);
    opt.set_parameter<std::string>("model_type", "RBDL");
    xbot_model_ = XBot::ModelInterface::getModel(opt);

    // Set home position defined in the srdf
    xbot_model_->getRobotState("home", qhome_);
    xbot_model_->setJointPosition(qhome_);

    // Those are associated to the SRDF model
    // FIXME, modify IDProblem to contain a map
    contact_links_.resize(4);
    contact_links_[0] = "lf_foot";
    contact_links_[1] = "rf_foot";
    contact_links_[2] = "lh_foot";
    contact_links_[3] = "rh_foot";

    tasks_pose_["com"] = Eigen::Affine3d::Identity();
    desired_tasks_pose_["com"] = Eigen::Affine3d::Identity();
    des_lf_foot_pose_ = tasks_pose_["lf_foot"] = desired_tasks_pose_["lf_foot"] = Eigen::Affine3d::Identity();
    des_rf_foot_pose_ = tasks_pose_["rf_foot"] = desired_tasks_pose_["rf_foot"] = Eigen::Affine3d::Identity();
    des_lh_foot_pose_ = tasks_pose_["lh_foot"] = desired_tasks_pose_["lh_foot"] = Eigen::Affine3d::Identity();
    des_rh_foot_pose_ = tasks_pose_["rh_foot"] = desired_tasks_pose_["rh_foot"] = Eigen::Affine3d::Identity();

    //desired_tasks_pose_.initRT(tasks_pose_);

    //id_prob_.reset(new OpenSoT::IDProblem(xbot_model_,DT,contact_links_));
    //fo_.reset(new OpenSoT::utils::ForceOptimization(xbot_model_,contact_links_,false));
    //id_prob_ = boost::make_shared<OpenSoT::IDProblem>(std::shared_ptr<XBot::ModelInterface>(xbot_model_, dt, contact_links_));

    // Resize the variables
    joint_positions_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_velocities_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_accellerations_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_efforts_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_positions_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_velocities_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_efforts_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    x_.resize(joint_states_.size()+FLOATING_BASE_DOFS);

    // Initializations
    joint_positions_.fill(0.0);
    joint_velocities_.fill(0.0);
    joint_accellerations_.fill(0.0);
    joint_efforts_.fill(0.0);
    com_position_.fill(0.0);
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.fill(0.0);
    x_.fill(0.0);
    des_com_position_.fill(0.0);
    floating_base_position_ = Eigen::Vector3d::Zero();
    floating_base_orientation_.normalize();
    floating_base_velocity_ = Eigen::Vector6d::Zero();
    floating_base_accelleration_ = Eigen::Vector6d::Zero();
    floating_base_pose_ = Eigen::Affine3d::Identity();
    contact_threshold_ = 0.03;
    swing_frequency_ = 1.5;

    // Create the realtime publishers
    ci_joint_states_rt_pub_ = new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "ci/joint_states", 4);
    ci_joint_states_rt_pub_->msg_.name.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.position.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.velocity.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.effort.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.name[0] = "x"; //FIXME
    ci_joint_states_rt_pub_->msg_.name[1] = "y";
    ci_joint_states_rt_pub_->msg_.name[2] = "z";
    ci_joint_states_rt_pub_->msg_.name[3] = "r";
    ci_joint_states_rt_pub_->msg_.name[4] = "p";
    ci_joint_states_rt_pub_->msg_.name[5] = "y";
    for (unsigned int i = 0; i < joint_names_.size(); i++)
        ci_joint_states_rt_pub_->msg_.name[i+FLOATING_BASE_DOFS] = joint_names_[i];

    state_estimation_rt_pub_ = new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "/state_estimation", 4);
    state_estimation_rt_pub_->msg_.header.frame_id = "world"; //FIXME
    state_estimation_rt_pub_->msg_.child_frame_id  = "base_link";

    tasks_actual_pose_rt_pub_ = new realtime_tools::RealtimePublisher<dls_controller::TasksPose>(controller_nh, "/tasks_actual_pose", 4);
    tasks_actual_pose_rt_pub_->msg_.reference_frame = "base_link";
    tasks_actual_pose_rt_pub_->msg_.tasks_name.resize(tasks_pose_.size());
    tasks_actual_pose_rt_pub_->msg_.tasks_pose.poses.resize(tasks_pose_.size());

    tasks_desired_pose_rt_pub_ = new realtime_tools::RealtimePublisher<dls_controller::TasksPose>(controller_nh, "/tasks_desired_pose", 4);
    tasks_desired_pose_rt_pub_->msg_.reference_frame = "world";
    tasks_desired_pose_rt_pub_->msg_.tasks_name.resize(desired_tasks_pose_.size());
    tasks_desired_pose_rt_pub_->msg_.tasks_pose.poses.resize(desired_tasks_pose_.size());

    // Reference for the tasks
    //tasks_desired_sub_ = controller_nh.subscribe("tasks_desired", 1, &Controller::setTasksDesired, this);

    // Spawn the odom publisher thread
    odom_publisher_thread_.reset(new std::thread(&Controller::odomPublisher,this));

    solver_reset_done_ = false;

    // Set the callback for the dynamic reconfigure server
    server_ = new dynamic_reconfigure::Server<dls_controller::DlsControllerConfig>(controller_nh);
    server_->setCallback( boost::bind(&Controller::dynamicReconfigureCallback, this, _1, _2));

    return true;
}

void Controller::dynamicReconfigureCallback(dls_controller::DlsControllerConfig &config, uint32_t level)
{
    switch(level)
    {
        case 0:
            toggleSolver();
            break;
        case 1:
            toggleTracking();
            break;
        case 2:
            setSwingFreq(config.swing_frequency);
            break;
        case 3:
            setContactThreshold(config.contact_threshold);
            break;
        case 4:
            setLambda("lf_foot",config.lf_foot_lambda);
            setLambda("rf_foot",config.rf_foot_lambda);
            setLambda("lh_foot",config.lh_foot_lambda);
            setLambda("rh_foot",config.rh_foot_lambda);
            setLambda("com",config.com_lambda);
            setLambda("waist",config.waist_lambda);
            break;
        default:
            break;
    }
}

bool Controller::setContactThreshold(const double contact_threshold)
{

     if(contact_threshold>=0.0)
     {
         ROS_INFO_STREAM_NAMED(CONTROLLER_NAME,"setContactThreshold: set the contact threshold to "<<contact_threshold);
         contact_threshold_ = contact_threshold;
         return true;
     }
     else
     {
         ROS_WARN_NAMED(CONTROLLER_NAME,"setContactThreshold: contact_threshold has to be positive or 0!");
         return false;
     }

}

bool Controller::setSwingFreq(const double swing_frequency)
{

     if(swing_frequency>=0.0)
     {
         ROS_INFO_STREAM_NAMED(CONTROLLER_NAME,"setSwingFreq: set the swing frequency to "<<swing_frequency);
         swing_frequency_ = swing_frequency;
         return true;
     }
     else
     {
         ROS_WARN_NAMED(CONTROLLER_NAME,"setSwingFreq: swing_frequency has to be positive or 0!");
         return false;
     }
}

bool Controller::setLambda(const std::string& task_name, const double lambda_value)
{

    if(task_name.empty())
    {
        ROS_WARN_NAMED(CONTROLLER_NAME,"setLambda: no task_name given!");
        return false;
    }

    if(id_prob_)
    {
        if(lambda_value>0.0)
        {
            // FIXME hardcoded like there is no tomorrow
            if(task_name == "com")
                id_prob_->_com->setLambda(lambda_value);
            else if(task_name == "waist")
                id_prob_->_waist->setLambda(lambda_value);
            else if(task_name == "lf_foot")
                id_prob_->_feet[0]->setLambda(lambda_value);
            else if(task_name == "rf_foot")
                id_prob_->_feet[1]->setLambda(lambda_value);
            else if(task_name == "lh_foot")
                id_prob_->_feet[2]->setLambda(lambda_value);
            else if(task_name == "rh_foot")
                id_prob_->_feet[3]->setLambda(lambda_value);
            else
            {
                ROS_WARN_NAMED(CONTROLLER_NAME,"setLambda: the selected task does not exist!");
                return false;
            }
            ROS_INFO_STREAM_NAMED(CONTROLLER_NAME,"setLambda: set "<<task_name<<" to lambda "<<lambda_value);
        }
        else
        {
            ROS_WARN_NAMED(CONTROLLER_NAME,"setLambda: lambda_value has to be positive!");
            return false;
        }
    }
    else
    {
        ROS_WARN_NAMED(CONTROLLER_NAME,"setLambda: problem not initialized yet! Did you toggleSolver first?");
        return false;
    }

    return true;
}

void Controller::toggleSolver()
{
    solver_started_=!solver_started_;

    if(solver_started_)
        ROS_INFO("Solver integration is ON");
    else
        ROS_INFO("Solver integration is OFF");
}

void Controller::toggleTracking()
{
    tracking_active_=!tracking_active_;

    if(tracking_active_)
        ROS_INFO("Tracking is ON");
    else
        ROS_INFO("Tracking is OFF");
}

void Controller::readImu()
{
    // FIXME For now we select the first imu
    unsigned int selected_imu = 0;
    imu_accelerometer_ = Eigen::Map<const Eigen::Vector3d>(imu_sensors_[selected_imu].getLinearAcceleration());
    imu_gyroscope_ = Eigen::Map<const Eigen::Vector3d>(imu_sensors_[selected_imu].getAngularVelocity());
    imu_orientation_.w() = imu_sensors_[selected_imu].getOrientation()[0];
    imu_orientation_.x() = imu_sensors_[selected_imu].getOrientation()[1];
    imu_orientation_.y() = imu_sensors_[selected_imu].getOrientation()[2];
    imu_orientation_.z() = imu_sensors_[selected_imu].getOrientation()[3];
}

void Controller::readJoints()
{
    joint_positions_.setZero(joint_positions_.size());
    joint_velocities_.setZero(joint_positions_.size());
    joint_accellerations_.setZero(joint_positions_.size());
    joint_efforts_.setZero(joint_positions_.size());

    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        joint_positions_(i+FLOATING_BASE_DOFS) = joint_states_[i].getPosition();
        joint_velocities_(i+FLOATING_BASE_DOFS) = joint_states_[i].getVelocity();
        joint_accellerations_(i+FLOATING_BASE_DOFS) = 0.0; // FIXME
        joint_efforts_(i+FLOATING_BASE_DOFS) = joint_states_[i].getEffort();
    }
}

void Controller::stateEstimation()
{
    // FIXME For now we select the first state estimator
    unsigned int selected_se = 0;
    floating_base_position_ = Eigen::Map<const Eigen::Vector3d>(state_estimators_[selected_se].getLinearPosition());

    floating_base_orientation_.w() = state_estimators_[selected_se].getOrientation()[0];
    floating_base_orientation_.x() = state_estimators_[selected_se].getOrientation()[1];
    floating_base_orientation_.y() = state_estimators_[selected_se].getOrientation()[2];
    floating_base_orientation_.z() = state_estimators_[selected_se].getOrientation()[3];

    floating_base_velocity_.segment(0,3) = Eigen::Map<const Eigen::Vector3d>(state_estimators_[selected_se].getLinearVelocity());
    floating_base_accelleration_.segment(0,3) = Eigen::Map<const Eigen::Vector3d>(state_estimators_[selected_se].getLinearAcceleration());

    floating_base_velocity_.segment(3,3) = Eigen::Map<const Eigen::Vector3d>(state_estimators_[selected_se].getAngularVelocity());
    floating_base_accelleration_.segment(3,3) = Eigen::Map<const Eigen::Vector3d>(state_estimators_[selected_se].getAngularAcceleration());

    floating_base_pose_.translation() = floating_base_position_;
    floating_base_pose_.linear() = floating_base_orientation_.normalized().toRotationMatrix();
}

void Controller::updateXBotModel()
{
    xbot_model_->setJointVelocity(joint_velocities_);
    xbot_model_->setJointPosition(joint_positions_);
    xbot_model_->setFloatingBaseState(floating_base_pose_,floating_base_velocity_);
    //xbot_model_->setFloatingBaseOrientation(imu_orientation_.normalized().toRotationMatrix().transpose());
    //xbot_model_->setFloatingBasePose(floating_base_pose_);
    if(id_prob_)
    {
        id_prob_->_com->getActualPose(com_position_); // Note: we use com_position_ as a tmp object!
        tasks_pose_["com"].translation() = com_position_;
        id_prob_->_feet[0]->getActualPose(tasks_pose_["lf_foot"]);
        id_prob_->_feet[1]->getActualPose(tasks_pose_["rf_foot"]);
        id_prob_->_feet[2]->getActualPose(tasks_pose_["lh_foot"]);
        id_prob_->_feet[3]->getActualPose(tasks_pose_["rh_foot"]);

        id_prob_->_com->getReference(com_position_);
        desired_tasks_pose_["com"].translation() = com_position_;
        id_prob_->_feet[0]->getReference(desired_tasks_pose_["lf_foot"]);
        id_prob_->_feet[1]->getReference(desired_tasks_pose_["rf_foot"]);
        id_prob_->_feet[2]->getReference(desired_tasks_pose_["lh_foot"]);
        id_prob_->_feet[3]->getReference(desired_tasks_pose_["rh_foot"]);
    }

}

void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting DLS Controller");

    // Read from the hardware interfaces:
    // 1) Joints
    readJoints();
    // 2) IMU
    readImu();
    // 3) State Estimation
    stateEstimation();
    // 4) Virtual Model Update
    updateXBotModel();

    ROS_DEBUG("Starting DLS Controller Completed");
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{

    // Read from the hardware interfaces:
    // 1) Joints
    readJoints();
    // 2) IMU
    readImu();
    // 3) State Estimation
    stateEstimation();
    // 4) Virtual Model Update
    updateXBotModel();

    int cnt = 0;
    for(unsigned int i = 0; i < contact_sensors_.size(); i++)
        if(!*contact_sensors_[i].getContactState())
            cnt++;
    if(cnt == 4)
         ROS_INFO("I am Flying!");

    // Set Default values
    //des_com_position_ << -0.05, -0.02, 0.5; //w.r.t to the world
    des_com_position_ << 0.0, 0.0, 0.5; //w.r.t to the world
    des_joint_positions_ = qhome_;

    if(solver_started_) // Use the ID solver to calculate the torques
    {
        if(!solver_reset_done_)
        {
            ROS_INFO("Reset the solver");
            id_prob_.reset(new OpenSoT::IDProblem(xbot_model_,period.toSec(),contact_links_)); // FIXME NO-RT
            solver_reset_done_ = true;

            // Set the initial feet poses
            id_prob_->_feet[0]->getActualPose(init_lf_foot_pose_ );
            id_prob_->_feet[1]->getActualPose(init_rf_foot_pose_ );
            id_prob_->_feet[2]->getActualPose(init_lh_foot_pose_ );
            id_prob_->_feet[3]->getActualPose(init_rh_foot_pose_ );
            id_prob_->_com->getActualPose(    init_com_position_);

        }

        // Always track the com position
        //des_com_position_ = init_com_position_;
        id_prob_->_com->setReference(des_com_position_);

        if(tracking_active_)
        {
            // Compute the periodic swing
            time_ += period.toSec();
            const double angle = 2.0 * M_PI * (swing_frequency_ * time_);
            const double amp = 0.1;
            const double z_lf_rh = amp/2.0 * (1 - std::cos(angle));
            const double z_rf_lh = amp/2.0 * (1 - std::cos(angle+M_PI));

            // Set the contacts for the solver
            if(z_lf_rh >= contact_threshold_)
            {
                id_prob_->_wrenches_lims->getWrenchLimits("lf_foot")->releaseContact(true);
                id_prob_->_wrenches_lims->getWrenchLimits("rh_foot")->releaseContact(true);

                id_prob_->_wrenches_lims->getWrenchLimits("rf_foot")->releaseContact(false);
                id_prob_->_wrenches_lims->getWrenchLimits("lh_foot")->releaseContact(false);
            }
            else
            {
                id_prob_->_wrenches_lims->getWrenchLimits("lf_foot")->releaseContact(false);
                id_prob_->_wrenches_lims->getWrenchLimits("rh_foot")->releaseContact(false);

                id_prob_->_wrenches_lims->getWrenchLimits("rf_foot")->releaseContact(true);
                id_prob_->_wrenches_lims->getWrenchLimits("lh_foot")->releaseContact(true);
            }

            // Fix the feet to an initial pose
            des_lf_foot_pose_ = init_lf_foot_pose_;
            des_rf_foot_pose_ = init_rf_foot_pose_;
            des_lh_foot_pose_ = init_lh_foot_pose_;
            des_rh_foot_pose_ = init_rh_foot_pose_;

            des_lf_foot_pose_.translation().z() = des_lf_foot_pose_.translation().z() + z_lf_rh;
            des_rh_foot_pose_.translation().z() = des_rh_foot_pose_.translation().z() + z_lf_rh;
            des_rf_foot_pose_.translation().z() = des_rf_foot_pose_.translation().z() + z_rf_lh;
            des_lh_foot_pose_.translation().z() = des_lh_foot_pose_.translation().z() + z_rf_lh;

            // Set the targets for the feet
            id_prob_->_feet[0]->setReference(des_lf_foot_pose_);
            id_prob_->_feet[1]->setReference(des_rf_foot_pose_);
            id_prob_->_feet[2]->setReference(des_lh_foot_pose_);
            id_prob_->_feet[3]->setReference(des_rh_foot_pose_);

        }
        else
        {
            id_prob_->_wrenches_lims->getWrenchLimits("lf_foot")->releaseContact(false);
            id_prob_->_wrenches_lims->getWrenchLimits("rh_foot")->releaseContact(false);
            id_prob_->_wrenches_lims->getWrenchLimits("rf_foot")->releaseContact(false);
            id_prob_->_wrenches_lims->getWrenchLimits("lh_foot")->releaseContact(false);

            id_prob_->_feet[0]->setReference(init_lf_foot_pose_);
            id_prob_->_feet[1]->setReference(init_rf_foot_pose_);
            id_prob_->_feet[2]->setReference(init_lh_foot_pose_);
            id_prob_->_feet[3]->setReference(init_rh_foot_pose_);
        }

        // Solver Update
        id_prob_->update();

        // Solve ID
        x_ = des_joint_efforts_; // Store the old desired efforts, to apply in case the solver gets mad
        if(!id_prob_->solve(x_))
        {
            ROS_ERROR("OpenSoT::IDProblem: unable to solve");
        }
        else
            des_joint_efforts_ = x_;

        pid_active_ = false;
    }
    else // Use a position PID controller
    {
        pid_active_ = true;
    }

    if(pid_active_)
    {
        for (unsigned int i = 0; i < des_joint_p_gain_.size(); i++)
        {
            des_joint_p_gain_[i] = joint_p_gain_[i];
            des_joint_i_gain_[i] = joint_i_gain_[i];
            des_joint_d_gain_[i] = joint_d_gain_[i];
        }
    }
    else
    {
        for (unsigned int i = 0; i < des_joint_p_gain_.size(); i++)
        {
            des_joint_p_gain_[i] = 0.0;
            des_joint_i_gain_[i] = 0.0;
            des_joint_d_gain_[i] = 0.0;
        }
    }

    if(pid_active_)
        des_joint_efforts_.fill(0.0);

    // Write to the hardware interface
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        joint_states_[i].setCommandEffort(des_joint_efforts_(i+FLOATING_BASE_DOFS));
        joint_states_[i].setCommandPosition(des_joint_positions_(i+FLOATING_BASE_DOFS));
        joint_states_[i].setCommandVelocity(0.0);
        joint_states_[i].setCommandGains(des_joint_p_gain_[i],des_joint_i_gain_[i],des_joint_d_gain_[i]); //Set Gains P I D
    }

    // Publish
    if(ci_joint_states_rt_pub_->trylock())
    {
        for(unsigned int i = 0; i < joint_positions_.size(); i++)
        {
            ci_joint_states_rt_pub_->msg_.position[i]  = des_joint_positions_(i);
            ci_joint_states_rt_pub_->msg_.velocity[i]  = des_joint_velocities_(i);
            ci_joint_states_rt_pub_->msg_.effort[i]    = des_joint_efforts_(i);
            ci_joint_states_rt_pub_->msg_.header.stamp = time;
            ci_joint_states_rt_pub_->unlockAndPublish();
        }
    }
    if(state_estimation_rt_pub_->trylock())
    {
        state_estimation_rt_pub_->msg_.pose.pose.position.x     = floating_base_position_(0);
        state_estimation_rt_pub_->msg_.pose.pose.position.y     = floating_base_position_(1);
        state_estimation_rt_pub_->msg_.pose.pose.position.z     = floating_base_position_(2);
        state_estimation_rt_pub_->msg_.pose.pose.orientation.w  = floating_base_orientation_.w();
        state_estimation_rt_pub_->msg_.pose.pose.orientation.x  = floating_base_orientation_.x();
        state_estimation_rt_pub_->msg_.pose.pose.orientation.y  = floating_base_orientation_.y();
        state_estimation_rt_pub_->msg_.pose.pose.orientation.z  = floating_base_orientation_.z();

        state_estimation_rt_pub_->msg_.twist.twist.linear.x     = floating_base_velocity_(0);
        state_estimation_rt_pub_->msg_.twist.twist.linear.y     = floating_base_velocity_(1);
        state_estimation_rt_pub_->msg_.twist.twist.linear.z     = floating_base_velocity_(2);
        state_estimation_rt_pub_->msg_.twist.twist.angular.x    = floating_base_velocity_(3);
        state_estimation_rt_pub_->msg_.twist.twist.angular.y    = floating_base_velocity_(4);
        state_estimation_rt_pub_->msg_.twist.twist.angular.z    = floating_base_velocity_(5);

        state_estimation_rt_pub_->msg_.header.stamp = time;
        state_estimation_rt_pub_->unlockAndPublish();
    }

    if(tasks_actual_pose_rt_pub_->trylock())
    {
        TasksPoseMap::iterator it;
        unsigned int idx = 0;
        for(it = tasks_pose_.begin(); it != tasks_pose_.end(); it++)
        {
            tasks_actual_pose_rt_pub_->msg_.tasks_name[idx] = it->first;
            tasks_actual_pose_rt_pub_->msg_.tasks_pose.poses[idx].position.x = it->second.translation().x();
            tasks_actual_pose_rt_pub_->msg_.tasks_pose.poses[idx].position.y = it->second.translation().y();
            tasks_actual_pose_rt_pub_->msg_.tasks_pose.poses[idx].position.z = it->second.translation().z();
            //FIXME Missing orientation
            idx++;
        }
        tasks_actual_pose_rt_pub_->msg_.tasks_pose.header.stamp = time;
        tasks_actual_pose_rt_pub_->unlockAndPublish();
    }

    if(tasks_desired_pose_rt_pub_->trylock())
    {
        TasksPoseMap::iterator it;
        unsigned int idx = 0;
        for(it = desired_tasks_pose_.begin(); it != desired_tasks_pose_.end(); it++)
        {
            tasks_desired_pose_rt_pub_->msg_.tasks_name[idx] = it->first;
            tasks_desired_pose_rt_pub_->msg_.tasks_pose.poses[idx].position.x = it->second.translation().x();
            tasks_desired_pose_rt_pub_->msg_.tasks_pose.poses[idx].position.y = it->second.translation().y();
            tasks_desired_pose_rt_pub_->msg_.tasks_pose.poses[idx].position.z = it->second.translation().z();
            //FIXME Missing orientation
            idx++;
        }
        tasks_desired_pose_rt_pub_->msg_.tasks_pose.header.stamp = time;
        tasks_desired_pose_rt_pub_->unlockAndPublish();
    }
}

/*void Controller::setTasksDesired(const dls_controller::TasksPose::ConstPtr& msg)
{
    TasksPoseMap* map_ptr = desired_tasks_pose_.readFromNonRT();
    Eigen::Affine3d tmp_affine;
    if(msg->tasks_name.size() == msg->tasks_pose.poses.size()) // Check msg consistency
    {
        for(unsigned int i = 0; i< msg->tasks_name.size(); i++)
        {
            if(map_ptr->find(msg->tasks_name[i]) == map_ptr->end()) // FIXME That's the reason why I should use a class
                ROS_WARN_STREAM("Task "<< msg->tasks_name[i] << " not available.");
            else
            {
                tmp_affine.translation() = Eigen::Vector3d(msg->tasks_pose.poses[i].position.x,
                                                           msg->tasks_pose.poses[i].position.y,
                                                           msg->tasks_pose.poses[i].position.z);
                tmp_affine.linear() = Eigen::Quaterniond(msg->tasks_pose.poses[i].orientation.w,
                                                         msg->tasks_pose.poses[i].orientation.x,
                                                         msg->tasks_pose.poses[i].orientation.y,
                                                         msg->tasks_pose.poses[i].orientation.z).normalized().toRotationMatrix();

                map_ptr->at(msg->tasks_name[i]) = tmp_affine;
            }
        }
    desired_tasks_pose_.writeFromNonRT(*map_ptr);
    }
    else
        ROS_WARN("Can not set the desired tasks reference, the name vector has a different size from the pose vector.");
}*/

void Controller::odomPublisher()
{
    ROS_INFO("Start the odomPublisher");

    Eigen::Affine3d base_pose, world_pose;
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    while(!stopping_)
    {
        // Get floating base
        // FIXME It causes solver's failures, probably it is caused by the call to the robot model inside this thread.
        //xbot_model_->getFloatingBasePose(base_pose); // FIXME Is it thread safe?
        base_pose = floating_base_pose_; //FIXME No Thread safe

        // Do the inverse of it
        world_pose = base_pose.inverse();
        position = world_pose.translation();
        quaternion = world_pose.linear();
        quaternion.normalize();

        // Create the tf transform between /ci/base_link and /ci/world_odom
        transform.setOrigin(tf::Vector3(position(0),position(1),position(2)));

        q.setX(quaternion.x());
        q.setY(quaternion.y());
        q.setZ(quaternion.z());
        q.setW(quaternion.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/base_link" , "/ci/world_odom" ));

        // Create the tf transform between /ci/base_link and /base_link
        transform.setOrigin(tf::Vector3(0,0,0));
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/base_link", "/base_link"));

        std::this_thread::sleep_for( std::chrono::milliseconds(4) );

    }
    ROS_INFO("Stop the odomPublisher");
}

void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping DLS Controller");

    stopping_ = true;
    odom_publisher_thread_->join();

    ROS_DEBUG("Stopping DLS Controller Completed");
}

} //namespace
