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
#define THREADS_SLEEP_TIME_ms 4
#define CONTROLLER_NAME "dls_controller"

Controller::Controller()
    :solver_started_(false)
    ,pid_active_(true)
    ,tracking_active_(false)
    ,relative_tasks_active_(false)
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
    // NOTE: do not confuse these with the contact_sensors
    feet_names_.resize(4);
    feet_names_[0] = "lf_foot";
    feet_names_[1] = "rf_foot";
    feet_names_[2] = "lh_foot";
    feet_names_[3] = "rh_foot";
    hips_names_.resize(4);
    hips_names_[0] = "lf_hipassembly";
    hips_names_[1] = "rf_hipassembly";
    hips_names_[2] = "lh_hipassembly";
    hips_names_[3] = "rh_hipassembly";

    task_poses_["waist"].first = desired_task_poses_["waist"].first = Eigen::Affine3d::Identity();
    task_poses_["waist"].second = desired_task_poses_["waist"].second = Eigen::Vector6d::Zero();
    base_frames_["waist"] = "world";
    for(unsigned int i=0;i<feet_names_.size();i++)
    {
        task_poses_[feet_names_[i]].first = desired_task_poses_[feet_names_[i]].first = Eigen::Affine3d::Identity();
        task_poses_[feet_names_[i]].second = desired_task_poses_[feet_names_[i]].second = Eigen::Vector6d::Zero();
        base_frames_[feet_names_[i]] = "world";
    }

    //desired_task_poses_.initRT(task_poses_);

    //id_prob_.reset(new OpenSoT::IDProblem(xbot_model_,DT,feet_names_));
    //fo_.reset(new OpenSoT::utils::ForceOptimization(xbot_model_,feet_names_,false));
    //id_prob_ = boost::make_shared<OpenSoT::IDProblem>(std::shared_ptr<XBot::ModelInterface>(xbot_model_, dt, feet_names_));

    // Resize the variables
    joint_positions_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_velocities_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_accellerations_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_efforts_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_positions_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_velocities_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_efforts_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    x_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    normals_.resize(contact_sensors_.size());
    contacts_.resize(contact_sensors_.size());
    contact_forces_.resize(contact_sensors_.size());

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

    // Create the realtime publishers
    ci_joint_states_rt_pub_ = new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "ci/joint_states", 4);
    ci_joint_states_rt_pub_->msg_.name.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.position.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.velocity.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.effort.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    ci_joint_states_rt_pub_->msg_.name[0] = "x";
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

    tasks_actual_pose_rt_pub_ = new realtime_tools::RealtimePublisher<dls_controller::TaskPoses>(controller_nh, "/tasks_actual_pose", 4);
    tasks_actual_pose_rt_pub_->msg_.reference_frames.resize(task_poses_.size());
    tasks_actual_pose_rt_pub_->msg_.task_names.resize(task_poses_.size());
    tasks_actual_pose_rt_pub_->msg_.task_poses.poses.resize(task_poses_.size());
    tasks_actual_pose_rt_pub_->msg_.task_velocities.resize(task_poses_.size());

    tasks_desired_pose_rt_pub_ = new realtime_tools::RealtimePublisher<dls_controller::TaskPoses>(controller_nh, "/tasks_desired_pose", 4);
    tasks_desired_pose_rt_pub_->msg_.reference_frames.resize(desired_task_poses_.size());
    tasks_desired_pose_rt_pub_->msg_.task_names.resize(desired_task_poses_.size());
    tasks_desired_pose_rt_pub_->msg_.task_poses.poses.resize(desired_task_poses_.size());
    tasks_desired_pose_rt_pub_->msg_.task_velocities.resize(desired_task_poses_.size());

    contacts_rt_pub_ = new realtime_tools::RealtimePublisher<std_msgs::Int16MultiArray>(controller_nh, "/contacts", 4);
    contacts_rt_pub_->msg_.data.resize(4);

    // Reference for the tasks
    //tasks_desired_sub_ = controller_nh.subscribe("tasks_desired", 1, &Controller::setTasksDesired, this);

    for(unsigned int i=0;i<feet_names_.size();i++)
        visual_tools_[feet_names_[i]].reset(new rviz_visual_tools::RvizVisualTools("ci/world_odom",feet_names_[i]+"_rviz_visual_marker"));

    solver_reset_done_ = false;

    // Set the callback for the dynamic reconfigure server
    server_ = new dynamic_reconfigure::Server<dls_controller::DlsControllerConfig>(controller_nh);
    server_->setCallback( boost::bind(&Controller::dynamicReconfigureCallback, this, _1, _2));

    gait_generator_.reset(new GaitGenerator(0.8,feet_names_,hips_names_,"crawl","ellipse"));

    // Spawn the odom publisher thread
    odom_publisher_thread_.reset(new std::thread(&Controller::odomPublisher,this));
    // Spawn the rviz publisher thread
    rviz_publisher_thread_.reset(new std::thread(&Controller::rvizPublisher,this));

    cmds_.reset(new CommandsInterface(gait_generator_,xbot_model_));
    joy_handler_.reset(new JoyHandler(controller_nh,cmds_));

    return true;
}

void Controller::dynamicReconfigureUpdate()
{
    // Update the config for dynamic reconfigure
    if(id_prob_)
    {
        default_config_.com_lambda = id_prob_->_com->getLambda();
        default_config_.waist_lambda = id_prob_->_waist->getLambda();
        default_config_.postural_lambda = id_prob_->_postural->getLambda();
    }
    default_config_.toggle_solver = solver_started_;
    default_config_.toggle_relative_tasks = relative_tasks_active_;
    default_config_.toggle_tracking = tracking_active_;
    if(gait_generator_)
    {
        default_config_.gaits = gait_generator_->getGaitType();
        default_config_.swing_frequency = gait_generator_->getSwingFrequency(feet_names_[0]); // HACK
        default_config_.duty_cycle = gait_generator_->getDutyCycle(feet_names_[0]); // HACK
    }
    if(cmds_)
    {
        default_config_.base_max_linear_vel = cmds_->getMaxLinearVelocity();
        default_config_.base_max_angular_vel = cmds_->getMaxAngularVelocity();
    }
    if(server_)
        server_->updateConfig(default_config_);
}

void Controller::dynamicReconfigureCallback(dls_controller::DlsControllerConfig &config, uint32_t level)
{
    switch(level)
    {
    case 0:
        toggleSolver();
        break;
    case 1:
        toggleRelativeTasks();
        break;
    case 2:
        toggleTracking();
        break;
    case 3:
        setDutyCycle(config.duty_cycle);
        break;
    case 4:
        setLambda("com",config.com_lambda);
        setLambda("waist",config.waist_lambda);
        setLambda("postural",config.postural_lambda);
        break;
    case 5:
        setGaitType(config.gaits);
        break;
    case 6:
        setSwingFrequency(config.swing_frequency);
        break;
    case 7:
        cmds_->setMaxLinearVelocity(config.base_max_linear_vel);
        ROS_INFO_STREAM_NAMED(CONTROLLER_NAME,"Set maximum velocity to "<< config.base_max_linear_vel);
        break;
    case 8:
        cmds_->setMaxAngularVelocity(config.base_max_angular_vel);
        ROS_INFO_STREAM_NAMED(CONTROLLER_NAME,"Set maximum angular rate to "<< config.base_max_angular_vel);
        break;
    default:
        break;
    }
}

bool Controller::setSwingFrequency(const double& swing_frequency)
{
    if(gait_generator_)
        for(unsigned int i=0; i < feet_names_.size(); i++)
            gait_generator_->setSwingFrequency(feet_names_[i],swing_frequency);
    else
    {
        ROS_WARN_NAMED(CONTROLLER_NAME,"gait_generator not initialized yet.");
        return false;
    }
    return true;
}

bool Controller::setGaitType(const std::string& gait_type)
{
    try
    {
        if(gait_generator_)
            gait_generator_->setGaitType(gait_type);
        else
        {
            ROS_WARN_NAMED(CONTROLLER_NAME,"gait_generator not initialized yet.");
            return false;
        }
    }
    catch(...)
    {
        ROS_ERROR_NAMED(CONTROLLER_NAME,"Wrong gait!");
        return false;
    }
    return true;
}

bool Controller::setDutyCycle(const double& duty_cycle)
{
    if(duty_cycle>=0.0 && duty_cycle<=1.0 && gait_generator_)
    {
        gait_generator_->setDutyCycle(duty_cycle);
        ROS_INFO_STREAM_NAMED(CONTROLLER_NAME,"setDutyCycle: set the duty cycle to "<<duty_cycle);
        return true;
    }
    else
    {
        ROS_WARN_NAMED(CONTROLLER_NAME,"setDutyCycle: duty cycle has to be between 0 and 1!");
        return false;
    }
}

bool Controller::setLambda(const std::string& task_name, const double& lambda_value)
{

    if(task_name.empty())
    {
        ROS_WARN_NAMED(CONTROLLER_NAME,"setLambda: no task_name given!");
        return false;
    }

    if(id_prob_)
    {
        if(lambda_value>=0.0)
        {
            // FIXME hardcoded like there is no tomorrow
            if(task_name == "com")
                id_prob_->_com->setLambda(lambda_value);
            else if(task_name == "postural")
                id_prob_->_postural->setLambda(lambda_value);
            else if(task_name == "waist")
                id_prob_->_waist->setLambda(lambda_value);
            else if(task_name == "feet")
            {
                id_prob_->_feet["lf_foot"]->setLambda(lambda_value);
                id_prob_->_feet["rf_foot"]->setLambda(lambda_value);
                id_prob_->_feet["lh_foot"]->setLambda(lambda_value);
                id_prob_->_feet["rh_foot"]->setLambda(lambda_value);
            }
            else if(task_name == "lf_foot")
                id_prob_->_feet[task_name]->setLambda(lambda_value);
            else if(task_name == "rf_foot")
                id_prob_->_feet[task_name]->setLambda(lambda_value);
            else if(task_name == "lh_foot")
                id_prob_->_feet[task_name]->setLambda(lambda_value);
            else if(task_name == "rh_foot")
                id_prob_->_feet[task_name]->setLambda(lambda_value);
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

void Controller::toggleRelativeTasks()
{
    relative_tasks_active_=!relative_tasks_active_;

    if(relative_tasks_active_)
        ROS_INFO("Relative tasks are ON");
    else
        ROS_INFO("Relative tasks are OFF");
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

    floating_base_position_ << 0.0,0.0, floating_base_position_(2); // Remove x and y from the state estimation
    floating_base_pose_.translation() = floating_base_position_;

    //floating_base_pose_.linear() = floating_base_orientation_.normalized().toRotationMatrix();
    //floating_base_orientation_rpy_ = floating_base_orientation_.normalized().toRotationMatrix().eulerAngles(0, 1, 2);

    floating_base_pose_.linear() = quatToRotMat(floating_base_orientation_.normalized()).transpose();
    floating_base_orientation_rpy_ = quatToRPY(floating_base_orientation_.normalized());
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
        /*id_prob_->_com->getActualPose(com_position_); // FIXME Note: we use com_position_ as a tmp object!
        task_poses_["com"].translation() = com_position_;
        id_prob_->_com->getReference(com_position_);
        desired_task_poses_["com"].translation() = com_position_;*/

        id_prob_->_waist->getActualPose(task_poses_["waist"].first);
        id_prob_->_waist->getReference(desired_task_poses_["waist"].first);
        base_frames_["waist"] = id_prob_->_waist->getBaseLink();
        for(unsigned int i=0; i<feet_names_.size(); i++)
        {
            id_prob_->_feet[feet_names_[i]]->getActualPose(task_poses_[feet_names_[i]].first);
            id_prob_->_feet[feet_names_[i]]->getActualTwist(task_poses_[feet_names_[i]].second);
            id_prob_->_feet[feet_names_[i]]->getReference(desired_task_poses_[feet_names_[i]].first,desired_task_poses_[feet_names_[i]].second);
            base_frames_[feet_names_[i]] = id_prob_->_feet[feet_names_[i]]->getBaseLink();
        }
    }
}

void Controller::readContactsState()
{
    for(unsigned int i=0; i<contact_sensors_.size(); i++)
    {
        normals_[i] = Eigen::Map<const Eigen::Vector3d>(contact_sensors_[i].getNormal());
        contacts_[i] = *contact_sensors_[i].getContactState();
        contact_forces_[i] = Eigen::Map<const Eigen::Vector3d>(contact_sensors_[i].getForce());
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
    // 3) Read contacts state
    readContactsState();
    // 4) State Estimation
    stateEstimation();
    // 5) Virtual Model Update
    updateXBotModel();

    ROS_DEBUG("Starting DLS Controller Completed");
}

void Controller::setInitialPose(const std::string& base_frame, const std::string& contact_name)
{
    if(id_prob_)
    {
        if(id_prob_->_feet[contact_name]->setBaseLink(base_frame))
        {
            id_prob_->_feet[contact_name]->update(Eigen::VectorXd(1));
            id_prob_->_feet[contact_name]->getActualPose(tmp_affine3d_);
            if(gait_generator_)
                gait_generator_->setInitialPose(contact_name,tmp_affine3d_);
        }
        else
            ROS_ERROR_STREAM("Can not set base link: "<<base_frame);
    }
}

void Controller::setInitialPose(const std::string& base_frame)
{
    for(unsigned int i = 0;i<feet_names_.size(); i++)
    {
        if(id_prob_->_feet[feet_names_[i]]->setBaseLink(base_frame))
        {
            id_prob_->_feet[feet_names_[i]]->update(Eigen::VectorXd(1));
        }
        else
            ROS_ERROR_STREAM("Can not set base link: "<<base_frame);
    }
    setInitialPose();
}

void Controller::setInitialPose()
{
    if(id_prob_)
    {
        for(unsigned int i = 0;i<feet_names_.size(); i++)
        {
            id_prob_->_feet[feet_names_[i]]->getActualPose(tmp_affine3d_);
            if(gait_generator_)
                gait_generator_->setInitialPose(feet_names_[i],tmp_affine3d_);
        }
    }
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // Read from the hardware interfaces:
    // 1) Joints
    readJoints();
    // 2) IMU
    readImu();
    // 3) Read contacts state
    readContactsState();
    // 4) State Estimation
    stateEstimation();
    // 5) Virtual Model Update
    updateXBotModel();

    // Set Default values
    des_joint_positions_ = qhome_;

    cmds_->update(period.toSec());

    if(cmds_->getCmd() == CommandsInterface::HOLD)
        tracking_active_ = false;
    else
        tracking_active_ = true;

    if(solver_started_) // Use the ID solver to calculate the torques
    {
        if(!solver_reset_done_)
        {
            ROS_INFO("Reset the solver");
            id_prob_.reset(new OpenSoT::IDProblem(xbot_model_,period.toSec(),feet_names_)); // FIXME NO-RT
            solver_reset_done_ = true;

            // Set the initial feet poses
            setInitialPose(); //w.r.t to the frame selected in IDProblem

            // We need to set these values here because the robot is starting in the air with the simulation. Be sure to start the solver
            // when the robot is grounded.
            cmds_->setBasePosition(floating_base_position_);
            cmds_->setDefaultBasePosition(floating_base_position_);
            cmds_->setBaseOrientation(floating_base_orientation_rpy_);
            cmds_->setDefaultBaseOrientation(floating_base_orientation_rpy_);

            dynamicReconfigureUpdate();
        }

        if(tracking_active_)
        {
            gait_generator_->activateSwing();

            // Set the task reference for the waist
            id_prob_->_waist->getReference(tmp_affine3d_);
            tmp_affine3d_.linear() = cmds_->getBaseRotationReference();
            tmp_affine3d_.translation().z() = cmds_->getBaseHeight();
            id_prob_->_waist->setReference(tmp_affine3d_);

            // Give to the gait_generator the contact status of the feet and the steps length
            for(unsigned int i = 0; i<feet_names_.size(); i++)
            {
#ifdef HAPTIC_CLOSED_LOOP
                gait_generator_->setContact(feet_names_[i],contacts_[i]); // Used to close the loop on the feet state machine with the haptic sensor
#endif
                gait_generator_->setStepLength(feet_names_[i],cmds_->getStepLength(feet_names_[i]));
                gait_generator_->setStepHeading(feet_names_[i],cmds_->getStepHeading(feet_names_[i]));
                gait_generator_->setStepHeight(feet_names_[i],cmds_->getStepHeight(feet_names_[i]));
                gait_generator_->setStepHeadingRate(feet_names_[i],cmds_->getStepHeadingRate(feet_names_[i]));
            }

            // Update the gait_generator
            gait_generator_->update(period.toSec());

            for(unsigned int i = 0; i<feet_names_.size(); i++)
            {

                id_prob_->_feet[feet_names_[i]]->setReference(Eigen::Affine3d::Identity(),gait_generator_->getReferenceDot(feet_names_[i]));

                // Set the wrench limits to enstablish the contacts
                if(gait_generator_->isSwinging(feet_names_[i]))
                {
                    id_prob_->_wrenches_lims->getWrenchLimits(feet_names_[i])->releaseContact(true);
                    ROS_DEBUG_STREAM("Swinging: "<< feet_names_[i]);
                }
                else
                {
                    id_prob_->_wrenches_lims->getWrenchLimits(feet_names_[i])->releaseContact(false);
                    ROS_DEBUG_STREAM("Stance: "<< feet_names_[i]);
                }

            }
        }
        else
        {

            gait_generator_->deactivateSwing();

            // Force the contact to each foot
            for(unsigned int i = 0; i<feet_names_.size(); i++)
                id_prob_->_wrenches_lims->getWrenchLimits(feet_names_[i])->releaseContact(false);
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
    publish(time,period);
}

void Controller::setRelativeTasks()
{
    for(unsigned int i=0; i<feet_names_.size(); i++)
        if(gait_generator_->isSwinging(feet_names_[i]))
            // Set the base frame of the swinging feet w.r.t the base_link.
        {
            if(gait_generator_->isLiftOff(feet_names_[i]))
            {
                setInitialPose("base_link",feet_names_[i]);
            }
        }

    // Set the base frame of the feet in stance w.r.t the foot in touchDown.
    // Set the base frame of the foot in touchDown w.r.t the base_link.
    for(unsigned int i=0; i<feet_names_.size(); i++)
        if(gait_generator_->isTouchDown(feet_names_[i]))
        {
            for(unsigned int j = 0; j<feet_names_.size(); j++)
            {
                if(j!=i)
                {
                    if(gait_generator_->isInStanceOrInit(feet_names_[j]))// Another foot is in stance
                    {
                        setInitialPose(feet_names_[i],feet_names_[j]);
                        ROS_INFO_STREAM("Set "<< feet_names_[j] << " to respect to " << feet_names_[i]);
                    }
                }
                else
                {
                    setInitialPose("base_link",feet_names_[j]);
                    ROS_INFO_STREAM("Set "<< feet_names_[j] << " to respect to base_link");
                }
            }
        }
}

void Controller::setWorldTasks()
{
    for(unsigned int i=0; i<feet_names_.size(); i++)
        if(gait_generator_->isTouchDown(feet_names_[i]))
            setInitialPose("world",feet_names_[i]);
}


void Controller::rvizPublisher()
{
    ROS_INFO("Start the rvizPublisher");

    unsigned int n_points = 30;
    std::vector<Eigen::Affine3d> poses(n_points);
    std::vector<geometry_msgs::Pose> path(n_points);
    geometry_msgs::Point arrow_start;
    geometry_msgs::Point arrow_end;

    // Publish using rviz visual tools
    while(!stopping_)
    {
        if(id_prob_ && visual_tools_.size()>0)
        {
            for(unsigned int i=0; i<feet_names_.size();i++)
            {
                if(gait_generator_->isSwinging(feet_names_[i]))
                {
                    gait_generator_->getTrajectoryPreview(feet_names_[i],poses);
                    //visual_tools_->setBaseFrame(id_prob_->_feet[feet_names_[i]]->getBaseLink());

                    //gait_generator_->getTrajectoryPtr(feet_names_[i])->preview(poses);
                    for(unsigned int j=0; j<n_points; j++)
                    {
                        path[j].position.x = poses[j].translation().x();
                        path[j].position.y = poses[j].translation().y();
                        path[j].position.z = poses[j].translation().z();
                        //colors[j] = rviz_visual_tools::RED;
                    }
                    visual_tools_[feet_names_[i]]->publishPath(path);

                }
                else
                {
                    visual_tools_[feet_names_[i]]->deleteAllMarkers();
                }

                //visual_tools_[feet_names_[i]]->publishArrow(arrow_start,arrow_end);

                visual_tools_[feet_names_[i]]->trigger();
            }


        }

        std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
    }
    ROS_INFO("Stop the rvizPublisher");
}

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

        std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );

    }
    ROS_INFO("Stop the odomPublisher");
}

void Controller::publish(const ros::Time& time, const ros::Duration& period)
{

    // Publish using rt-publishers
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
        TaskPosesMap::iterator it;
        unsigned int idx = 0;
        for(it = task_poses_.begin(); it != task_poses_.end(); it++)
        {
            tasks_actual_pose_rt_pub_->msg_.reference_frames[idx] = base_frames_[it->first];
            tasks_actual_pose_rt_pub_->msg_.task_names[idx] = it->first;
            // Pose
            tasks_actual_pose_rt_pub_->msg_.task_poses.poses[idx].position.x = it->second.first.translation().x();
            tasks_actual_pose_rt_pub_->msg_.task_poses.poses[idx].position.y = it->second.first.translation().y();
            tasks_actual_pose_rt_pub_->msg_.task_poses.poses[idx].position.z = it->second.first.translation().z();
            // Twist
            tasks_actual_pose_rt_pub_->msg_.task_velocities[idx].linear.x = it->second.second(0);
            tasks_actual_pose_rt_pub_->msg_.task_velocities[idx].linear.y = it->second.second(1);
            tasks_actual_pose_rt_pub_->msg_.task_velocities[idx].linear.z = it->second.second(2);
            //FIXME Missing orientation
            idx++;
        }
        tasks_actual_pose_rt_pub_->msg_.task_poses.header.stamp = time;
        tasks_actual_pose_rt_pub_->unlockAndPublish();
    }

    if(tasks_desired_pose_rt_pub_->trylock())
    {
        TaskPosesMap::iterator it;
        unsigned int idx = 0;
        for(it = desired_task_poses_.begin(); it != desired_task_poses_.end(); it++)
        {
            tasks_desired_pose_rt_pub_->msg_.reference_frames[idx] = base_frames_[it->first];
            tasks_desired_pose_rt_pub_->msg_.task_names[idx] = it->first;
            // Pose
            tasks_desired_pose_rt_pub_->msg_.task_poses.poses[idx].position.x = it->second.first.translation().x();
            tasks_desired_pose_rt_pub_->msg_.task_poses.poses[idx].position.y = it->second.first.translation().y();
            tasks_desired_pose_rt_pub_->msg_.task_poses.poses[idx].position.z = it->second.first.translation().z();
            // Twist
            tasks_desired_pose_rt_pub_->msg_.task_velocities[idx].linear.x = it->second.second(0);
            tasks_desired_pose_rt_pub_->msg_.task_velocities[idx].linear.y = it->second.second(1);
            tasks_desired_pose_rt_pub_->msg_.task_velocities[idx].linear.z = it->second.second(2);
            //FIXME Missing orientation
            idx++;
        }
        tasks_desired_pose_rt_pub_->msg_.task_poses.header.stamp = time;
        tasks_desired_pose_rt_pub_->unlockAndPublish();
    }

    if(contacts_rt_pub_->trylock())
    {
        for(unsigned int i=0; i<contacts_rt_pub_->msg_.data.size(); i++)
            contacts_rt_pub_->msg_.data[i] = (*contact_sensors_[i].getContactState() ? 1 : 0);
        contacts_rt_pub_->unlockAndPublish();
    }
}

void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping DLS Controller");

    stopping_ = true;
    odom_publisher_thread_->join();
    rviz_publisher_thread_->join();

    ROS_DEBUG("Stopping DLS Controller Completed");
}

} //namespace
