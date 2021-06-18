/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief WholeBody Controller.
 *
 * This file contains the constructor, destructor, init, stopping and other facilities for the
 * WholeBody Controller.
 * @see todo.git
 */

#include <wb_controller/controller.h>

using namespace XBot;
using namespace Cartesian;
using namespace rt_logger;

namespace wb_controller {

#define CLASS_NAME "Controller"

std::vector<std::string> _dof_names = {}; // To be loaded from the robot model
std::vector<std::string> _cartesian_names = {"x","y","z","roll","pitch","yaw"}; // This is our standard cartesian dofs order
std::vector<std::string> _joints_prefix = {"haa","hfe","kfe"};

Controller::Controller()
    :solver_created_(false)
    ,solver_started_(false)
    ,init_done_(false)
    ,pid_active_(true)
    ,inertia_compensation_active_(true)
    ,stopping_(false)
{
}

Controller::~Controller()
{
}

bool Controller::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    ROS_DEBUG_NAMED(CLASS_NAME,"Initialize");

    nh_ = controller_nh;

    assert(robot_hw);

    hardware_interface::EffortJointInterface* jt_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
    hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
    hardware_interface::GroundTruthInterface* gt_hw = robot_hw->get<hardware_interface::GroundTruthInterface>();

    // Hardware interfaces checks
    if(!jt_hw)
    {
        ROS_ERROR("hardware_interface::EffortJointInterface not found");
        return false;
    }
    if(!imu_hw)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::ImuSensorInterface not found");
        return false;
    }
    if(!gt_hw)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::GroundTruthInterface not found");
        return false;
    }
    else
        ground_truth_ = gt_hw->getHandle("ground_truth");

    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR_NAMED(CLASS_NAME,"No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    if (!controller_nh.getParam("feet", feet_names_))
    {
        ROS_ERROR_NAMED(CLASS_NAME,"No feet given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    if (!controller_nh.getParam("arm_tip", arm_tip_name_))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No arm tip name given in the namespace: %s, proceeding without using the arm.", controller_nh.getNamespace().c_str());
        arm_tip_name_ = std::string();
    }
    if (!controller_nh.getParam("hips", hips_names_))
    {
        ROS_ERROR_NAMED(CLASS_NAME,"No hips given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    if (!controller_nh.getParam("imu_sensor", imu_name_))
    {
        ROS_ERROR_NAMED(CLASS_NAME,"No imu_sensor given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }
    double default_duty_factor = 0.3;
    if (!controller_nh.getParam("default_duty_factor", default_duty_factor))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default duty factor given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_duty_factor);
    }
    double default_swing_frequency = 3.0; // [Hz]
    if (!controller_nh.getParam("default_swing_frequency", default_swing_frequency))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default swing frequency given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_swing_frequency);
    }
    double default_contact_threshold = 50.0; // [N]
    if (!controller_nh.getParam("default_contact_threshold", default_contact_threshold))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default contact threshold given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_contact_threshold);
    }
    double default_step_height = 0.05; // [m]
    if (!controller_nh.getParam("default_step_height", default_step_height))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default step height given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_step_height);
    }
    double max_step_height = 0.15; // [m]
    if (!controller_nh.getParam("max_step_height", max_step_height))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No max step height given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_step_height);
    }
    double max_step_length = 0.5; // [m]
    if (!controller_nh.getParam("max_step_length", max_step_length))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No max step length given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_step_length);
    }
    double default_base_linear_velocity = 0.5; // [m/s]
    if (!controller_nh.getParam("default_base_linear_velocity", default_base_linear_velocity))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default base linear velocity given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_linear_velocity);
    }
    double default_base_angular_velocity = 0.5; // [rad/s]
    if (!controller_nh.getParam("default_base_angular_velocity", default_base_angular_velocity))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No default base angular velocity given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_angular_velocity);
    }

    // Setting up joint handles:
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
            ROS_ERROR_NAMED(CLASS_NAME,"Error loading joint_states_");
            return false;
        }
    }
    assert(joint_states_.size()>0);

    try
    {
        ROS_DEBUG_STREAM("Found imu sensor: "<<imu_name_);
        imu_sensor_ = imu_hw->getHandle(imu_name_);
    }
    catch(...)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"Error loading imu_sensor_");
        return false;
    }

    des_joint_p_gain_.resize(joint_states_.size());
    des_joint_i_gain_.resize(joint_states_.size());
    des_joint_d_gain_.resize(joint_states_.size());
    joint_p_gain_.resize(joint_states_.size());
    joint_i_gain_.resize(joint_states_.size());
    joint_d_gain_.resize(joint_states_.size());

    pids_.resize(joint_states_.size());
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // Getting PID gains
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", joint_p_gain_[i]))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", joint_i_gain_[i]))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", joint_d_gain_[i]))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        // Check if the values are positive
        if(joint_p_gain_[i]<0.0 || joint_i_gain_[i]<0.0 || joint_d_gain_[i]<0.0)
        {
            ROS_ERROR("PID gains must be positive!");
            return false;
        }
        ROS_DEBUG("P value for joint %i is: %f",i,joint_p_gain_[i]);
        ROS_DEBUG("I value for joint %i is: %f",i,joint_i_gain_[i]);
        ROS_DEBUG("D value for joint %i is: %f",i,joint_d_gain_[i]);

        pids_[i].setGains(joint_p_gain_[i],joint_i_gain_[i],joint_d_gain_[i],0,0);
        //pids_[i].initDynamicReconfig(controller_nh); // FIXME change namespace for the pids
    }

    Kd_swing_leg_.setZero();
    Kp_swing_leg_.setZero();
    Kd_stance_leg_.setZero();
    Kp_stance_leg_.setZero();

    // Getting Kp and Kd gains
    for(unsigned int i=0; i<_joints_prefix.size(); i++)
    {
        if (!controller_nh.getParam("gains/kp_swing/" + _joints_prefix[i] , Kp_swing_leg_(i,i)))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No kp_swing_%s gain given in the namespace: %s. ",_joints_prefix[i].c_str(),controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/kd_swing/" + _joints_prefix[i] , Kd_swing_leg_(i,i)))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No kd_swing_%s gain given in the namespace: %s. ",_joints_prefix[i].c_str(),controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/kp_stance/" + _joints_prefix[i] , Kp_stance_leg_(i,i)))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No kp_stance_%s gain given in the namespace: %s. ",_joints_prefix[i].c_str(),controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/kd_stance/" + _joints_prefix[i] , Kd_stance_leg_(i,i)))
        {
            ROS_ERROR_NAMED(CLASS_NAME,"No kd_stance_%s gain given in the namespace: %s. ",_joints_prefix[i].c_str(),controller_nh.getNamespace().c_str());
            return false;
        }
        // Check if the values are positive
        if(Kp_swing_leg_(i,i)<0.0 || Kd_swing_leg_(i,i)<0.0 || Kp_stance_leg_(i,i)<0.0 || Kd_stance_leg_(i,i)<0.0)
        {
            ROS_ERROR("Kp and Kd gains must be positive!");
            return false;
        }
    }
    double default_clik_gain = 0.0; // Default value
    if (!controller_nh.getParam("gains/clik_gain", default_clik_gain))
    {
        ROS_WARN_NAMED(CLASS_NAME,"No clik_gain given in the namespace: %s, set 0 as default value ", controller_nh.getNamespace().c_str());
    }

    // Assume we are working with a dog
    if(hips_names_.size()!=N_LEGS)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"Wrong number of hips!");
        return false;
    }
    if(feet_names_.size()!=N_LEGS)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"Wrong number of feet!");
        return false;
    }
    hips_names_ = sortByLegName(hips_names_);
    feet_names_ = sortByLegName(feet_names_);

    // Create the ModelInterface from XBot
    XBot::ConfigOptions opt;
    std::string urdf, srdf, problem;

    if(!root_nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No robot_description given in namespace /");
        return false;
    }
    if(!root_nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No robot_semantic_description given in namespace /");
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
    std::string model_type = "RBDL";
    opt.set_parameter<std::string>("model_type", model_type);
    xbot_model_ = XBot::ModelInterface::getModel(opt);

    _dof_names = xbot_model_->getEnabledJointNames();

    // Initialize the inertia related matrices
    xbot_model_->getInertiaMatrix(M_);
    Mi_.setZero(M_.rows(), M_.cols());
    Kp_postural_.setZero(M_.rows(), M_.cols());
    Kd_postural_.setZero(M_.rows(), M_.cols());

    // Check if the joint names in the ROS config file are in the same order as the one in the virtual model:
    assert(_dof_names.size() == joint_names_.size()+FLOATING_BASE_DOFS);
    for(unsigned int i=0;i<joint_names_.size();i++)
        if(_dof_names[i+FLOATING_BASE_DOFS]!=joint_names_[i])
        {
            ROS_ERROR_STREAM_NAMED(CLASS_NAME,"Joint names in the robot model type "<<model_type<< " are not in the same order as in the ROS config file of the controller.");
            return false;
        }

    // Resize the variables
    joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_positions_xbot_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_velocities_xbot_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_velocities_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_velocities_filt_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_accellerations_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_efforts_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_velocities_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_efforts_solver_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_efforts_pids_.resize(static_cast<Eigen::Index>(joint_states_.size()));
    des_joint_efforts_.resize(static_cast<Eigen::Index>(joint_states_.size()));
    x_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_contact_forces_.resize(FLOATING_BASE_DOFS*N_LEGS); // 24 = 6 dofs * 4 leg

    // Initializations
    joint_positions_.fill(0.0);
    joint_velocities_.fill(0.0);
    joint_velocities_filt_.fill(0.0);
    joint_accellerations_.fill(0.0);
    joint_efforts_.fill(0.0);
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.fill(0.0);
    x_.fill(0.0);
    imu_orientation_.normalize();
    pid_scale_ = 1.0;

    gait_generator_.reset(new GaitGenerator(feet_names_,hips_names_,"crawl","ellipse"));
    gait_generator_->setSwingFrequency(default_swing_frequency);
    gait_generator_->setDutyFactor(default_duty_factor);

    cmds_.reset(new FootholdsPlanner(gait_generator_,xbot_model_));
    cmds_->setLinearVelocity(default_base_linear_velocity);
    cmds_->setAngularVelocity(default_base_angular_velocity);
    cmds_->setStepHeight(default_step_height);
    cmds_->setMaxStepHeight(max_step_height);
    cmds_->setMaxStepLength(max_step_length);

    state_estimator_.reset(new StateEstimator(gait_generator_,xbot_model_));
    state_estimator_->setContactThreshold(default_contact_threshold);

    kin_.reset(new LegsKinematics(gait_generator_,xbot_model_));
    kin_->setClikGain(default_clik_gain);
    kin_->activateBaseHeightControl();

    std::string estimation_position_type;
    if (!controller_nh.getParam("estimation_position_type", estimation_position_type))
        ROS_WARN_NAMED(CLASS_NAME,"No default estimation_position_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),state_estimator_->getPositionEstimationType().c_str());
    else
        state_estimator_->setPositionEstimationType(estimation_position_type);

    std::string estimation_orientation_type;
    if (!controller_nh.getParam("estimation_orientation_type", estimation_orientation_type))
        ROS_WARN_NAMED(CLASS_NAME,"No default estimation_orientation_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),state_estimator_->getOrientationEstimationType().c_str());
    else
        state_estimator_->setOrientationEstimationType(estimation_orientation_type);

    joy_handler_.reset(new JoyHandler(controller_nh,cmds_));
    joy_handler_->addButtonHandler(boost::bind(&Controller::toggleSolver,this),JoyHandler::START);
    joy_handler_->addButtonHandler(boost::bind(&GaitGenerator::switchGait,gait_generator_.get()),JoyHandler::SELECT);

    keyboard_handler_.reset(new TwistHandler(controller_nh,cmds_));

    // initialize the filters
    cutoff_hz_gyro_ = 300.;
    cutoff_hz_qdot_ = 300.;

    if(!root_nh.getParam("/task_period",period_)) // Get the initial task period
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No task period given in namespace /");
        return false;
    }

    qdot_filter_.setOmega(2.0*M_PI*cutoff_hz_qdot_);
    qdot_filter_.setDamping(1.0);
    qdot_filter_.setTimeStep(period_);

    imu_gyroscope_filter_.setOmega(2.0*M_PI*cutoff_hz_gyro_);
    imu_gyroscope_filter_.setDamping(1.0);
    imu_gyroscope_filter_.setTimeStep(period_);

    // Spawn the odom publisher thread
    odom_publisher_thread_.reset(new std::thread(&Controller::odomPublisher,this));

    initPublishers(root_nh,controller_nh);

    // Set the callback for the dynamic reconfigure server
    server_.reset(new dynamic_reconfigure::Server<wb_controller::controllerConfig>(controller_nh));
    server_->setCallback(boost::bind(&Controller::dynamicReconfigureCallback, this, _1, _2));

    RtLogger::getLogger().addPublisher(CLASS_NAME"/imu_gyroscope",imu_gyroscope_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/imu_gyroscope_filt",imu_gyroscope_filt_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/joint_velocities_",joint_velocities_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/joint_velocities_filt",joint_velocities_filt_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/des_joint_efforts_solver",des_joint_efforts_solver_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/des_joint_efforts_pids",des_joint_efforts_pids_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/des_joint_efforts",des_joint_efforts_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/des_base_rpy",des_base_rpy_);
    RtLogger::getLogger().addPublisher(CLASS_NAME"/period",period_);

    return true;
}

void Controller::dynamicReconfigureUpdate()
{

    // Update the config for dynamic reconfigure
    default_config_.toggle_solver = solver_started_;
    default_config_.toggle_inertia_compensation = inertia_compensation_active_;
    default_config_.pid_scale = pid_scale_;
    default_config_.cutoff_hz_qdot = cutoff_hz_qdot_;
    default_config_.cutoff_hz_gyro = cutoff_hz_gyro_;

    default_config_.kp_haa_swing   = Kp_swing_leg_(0,0);
    default_config_.kp_hfe_swing   = Kp_swing_leg_(1,1);
    default_config_.kp_kfe_swing   = Kp_swing_leg_(2,2);

    default_config_.kd_haa_swing   = Kd_swing_leg_(0,0);
    default_config_.kd_hfe_swing   = Kd_swing_leg_(1,1);
    default_config_.kd_kfe_swing   = Kd_swing_leg_(2,2);

    default_config_.kp_haa_stance = Kp_stance_leg_(0,0);
    default_config_.kp_hfe_stance = Kp_stance_leg_(1,1);
    default_config_.kp_kfe_stance = Kp_stance_leg_(2,2);

    default_config_.kd_haa_stance = Kd_stance_leg_(0,0);
    default_config_.kd_hfe_stance = Kd_stance_leg_(1,1);
    default_config_.kd_kfe_stance = Kd_stance_leg_(2,2);

    if(gait_generator_)
    {
        default_config_.gaits = gait_generator_->getGaitType();
        default_config_.swing_frequency = gait_generator_->getSwingFrequency(feet_names_[0]); // FIXME - HACK
        default_config_.duty_factor = gait_generator_->getDutyFactor(feet_names_[0]); // FIXME - HACK
    }
    if(cmds_)
    {
        default_config_.base_linear_vel = cmds_->getLinearVelocity();
        default_config_.base_angular_vel = cmds_->getAngularVelocity();
        default_config_.step_height = cmds_->getStepHeight();
    }
    if(state_estimator_)
        default_config_.contact_force_th = state_estimator_->getContactThreshold();

    if(kin_)
    {
        default_config_.toggle_base_height_control = kin_->isBaseHeightControlActive();
        default_config_.clik_gain = kin_->getClikGain();
    }

    if(server_)
        server_->updateConfig(default_config_);
}

void Controller::dynamicReconfigureCallback(wb_controller::controllerConfig &config, uint32_t level)
{
    switch(level)
    {
    case 0:
        toggleSolver();
        break;
    case 1:
        toggleBaseHeightControl();
        break;
    case 2:
        toggleInertiaCompensation();
        break;
    case 3:
        setDutyFactor(config.duty_factor);
        break;
    case 4:
        setGaitType(config.gaits);
        break;
    case 5:
        setSwingFrequency(config.swing_frequency);
        break;
    case 6:
        cmds_->setLinearVelocity(config.base_linear_vel);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set base linear velocity to "<< config.base_linear_vel);
        break;
    case 7:
        cmds_->setAngularVelocity(config.base_angular_vel);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set base angular velocity to "<< config.base_angular_vel);
        break;
    case 8:
        cmds_->setStepHeight(config.step_height);
        break;
    case 9:
        state_estimator_->setContactThreshold(config.contact_force_th);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set contact force threshold to "<< config.contact_force_th);
        break;
    case 10:
        pid_scale_ = config.pid_scale;
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set pid scale to "<< config.pid_scale);
        break;
    case 11:
        cutoff_hz_qdot_ = config.cutoff_hz_qdot;
        qdot_filter_.setOmega(2.0*M_PI*cutoff_hz_qdot_);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set cutoff frequency for qdot filter at "<< config.cutoff_hz_qdot);
        break;
    case 12:
        cutoff_hz_gyro_ = config.cutoff_hz_gyro;
        imu_gyroscope_filter_.setOmega(2.0*M_PI*cutoff_hz_gyro_);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set cutoff frequency  for gyroscope filter at "<< config.cutoff_hz_gyro);
        break;
    case 13:
        kin_->setClikGain(config.clik_gain);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set x err gain at "<< config.clik_gain);
        break;
    case 14:
        // FIXME: this is not thread safe!
        // Kp swing
        Kp_swing_leg_(0,0) = config.kp_haa_swing;
        Kp_swing_leg_(1,1) = config.kp_hfe_swing;
        Kp_swing_leg_(2,2) = config.kp_kfe_swing;
        // Kd swing
        Kd_swing_leg_(0,0) = config.kd_haa_swing;
        Kd_swing_leg_(1,1) = config.kd_hfe_swing;
        Kd_swing_leg_(2,2) = config.kd_kfe_swing;
        // Kp stance
        Kp_stance_leg_(0,0) = config.kp_haa_stance;
        Kp_stance_leg_(1,1) = config.kp_hfe_stance;
        Kp_stance_leg_(2,2) = config.kp_kfe_stance;
        // Kd stance
        Kd_stance_leg_(0,0) = config.kd_haa_stance;
        Kd_stance_leg_(1,1) = config.kd_hfe_stance;
        Kd_stance_leg_(2,2) = config.kd_kfe_stance;
        ROS_INFO_NAMED(CLASS_NAME,"Set Kp and Kd for the postural");
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
        ROS_WARN_NAMED(CLASS_NAME,"gait_generator not initialized yet.");
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
            ROS_WARN_NAMED(CLASS_NAME,"gait_generator not initialized yet.");
            return false;
        }
    }
    catch(...)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"Wrong gait!");
        return false;
    }
    return true;
}

bool Controller::setDutyFactor(const double& duty_factor)
{
    if(duty_factor>=0.0 && duty_factor<=1.0 && gait_generator_)
    {
        gait_generator_->setDutyFactor(duty_factor);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"setDutyFactor: set the duty factor to "<<duty_factor);
        return true;
    }
    else
    {
        ROS_WARN_NAMED(CLASS_NAME,"setDutyFactor: duty factor has to be between 0 and 1!");
        return false;
    }
}

void Controller::toggleBaseHeightControl()
{
    if(state_estimator_->getPositionEstimationType() == "estimated_z" || state_estimator_->getPositionEstimationType() == "ground_truth")
    {
        kin_->toggleBaseHeightControl();

        if(kin_->isBaseHeightControlActive())
            ROS_INFO("Base height control is ON");
        else
            ROS_INFO("Base height control is OFF");
    }
    else
    {
        kin_->deactivateBaseHeightControl();
        ROS_WARN_NAMED(CLASS_NAME,"Can not activate the base height control, the state estimator (%s) is not configured to do so!",state_estimator_->getPositionEstimationType().c_str());
    }
}

void Controller::toggleSolver()
{
    if(!solver_created_)
    {
        ROS_INFO("Reset the solver");
        id_prob_.reset(new OpenSoT::IDProblem(nh_,xbot_model_,feet_names_,arm_tip_name_));
        solver_created_ = true;
    }

    // Perform the init procedure
    init_done_ = false;

    solver_started_=!solver_started_;

    if(solver_started_)
        ROS_INFO("Solver integration is ON");
    else
        ROS_INFO("Solver integration is OFF");

}

void Controller::toggleInertiaCompensation()
{
    inertia_compensation_active_=!inertia_compensation_active_;

    if(inertia_compensation_active_)
        ROS_INFO("Inertia compensation is ON");
    else
        ROS_INFO("Inertia compensation is OFF");
}

void Controller::readJoints()
{
    joint_positions_.setZero(joint_positions_.size());
    joint_velocities_.setZero(joint_positions_.size());
    joint_velocities_filt_.setZero(joint_positions_.size());
    joint_accellerations_.setZero(joint_positions_.size());
    joint_efforts_.setZero(joint_positions_.size());

    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        joint_positions_(i+FLOATING_BASE_DOFS) = joint_states_[i].getPosition();
        joint_velocities_(i+FLOATING_BASE_DOFS) = joint_states_[i].getVelocity();
        joint_accellerations_(i+FLOATING_BASE_DOFS) = 0.0; // FIXME
        joint_efforts_(i+FLOATING_BASE_DOFS) = joint_states_[i].getEffort();
    }

    // Filter the qdot
    joint_velocities_filt_ = qdot_filter_.process(joint_velocities_);
}

void Controller::readImu()
{
    imu_accelerometer_ = Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getLinearAcceleration());
    imu_gyroscope_ = Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getAngularVelocity());
    imu_orientation_.w() = imu_sensor_.getOrientation()[0];
    imu_orientation_.x() = imu_sensor_.getOrientation()[1];
    imu_orientation_.y() = imu_sensor_.getOrientation()[2];
    imu_orientation_.z() = imu_sensor_.getOrientation()[3];

    // Filter the imu velocities
    imu_gyroscope_filt_ = imu_gyroscope_filter_.process(imu_gyroscope_);
}

void Controller::starting(const ros::Time&  /*time*/)
{
    ROS_DEBUG_NAMED(CLASS_NAME,"Starting the Controller");

    // Read from the hardware interfaces:
    // 1) Joints
    readJoints();
    // 2) IMU
    readImu();

    ROS_DEBUG_NAMED(CLASS_NAME,"Starting the Controller Completed");
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // Read from the hardware interfaces:
    // 1) Joints
    readJoints();
    // 2) IMU
    readImu();

    period_ = period.toSec();

    state_estimator_->setJointPosition(joint_positions_);
    state_estimator_->setJointVelocity(joint_velocities_filt_);
    state_estimator_->setJointEffort(joint_efforts_);
    if(state_estimator_->getPositionEstimationType() == "ground_truth")
    {
        state_estimator_->setGroundTruthBasePosition(Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearPosition()));
        state_estimator_->setGroundTruthBaseLinearVelocity(Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearVelocity()));
    }
    if(state_estimator_->getOrientationEstimationType() == "ground_truth")
    {
        state_estimator_->setGroundTruthBaseOrientation(imu_orientation_);
        state_estimator_->setGroundTruthBaseAngularVelocity(imu_gyroscope_filt_);
    }
    else
    {
        state_estimator_->setImuOrientation(imu_orientation_);
        state_estimator_->setImuGyroscope(imu_gyroscope_filt_);
    }

    state_estimator_->update(period.toSec());

    if(solver_started_) // Use the ID solver to calculate the torques
    {
        if(!init_done_) // FIXME Prepare a proper start up and rest procedure
        {
            // We need to set these values here because the robot is starting in the air with the simulation.
            // Be sure to start the solver and the contact estimation when the robot is grounded.
            state_estimator_->resetGyroscopeIntegration();
            state_estimator_->startContactsEstimation();
            state_estimator_->startHapticContactLoop();
            cmds_->setBasePosition(state_estimator_->getFloatingBasePosition());
            cmds_->setDefaultBasePosition(state_estimator_->getFloatingBasePosition());
            cmds_->setBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
            cmds_->setDefaultBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
            cmds_->initializeFeetPosition();

            // Reset the tasks
            //id_prob_->reset(); // FIXME

            kin_->reset();

            des_joint_positions_ = kin_->getJointHomePositions();
            des_joint_velocities_.fill(0.0);

            imu_gyroscope_filter_.setTimeStep(period_);
            qdot_filter_.setTimeStep(period_);

            dynamicReconfigureUpdate(); // FIXME Why is it here?

            init_done_ = true;

        }

        cmds_->update(period.toSec()); // FIXME This should be done only after pid_scale_ = 0

        // FIXME I should add something to the FootholdsPlanner!!!
        // Get the external reference (interactive marker) for the arm if available
        id_prob_->updateReference(arm_tip_name_);

        // Set the task reference for the waist
        id_prob_->_waistRPY->getReference(tmp_affine3d_);
        tmp_affine3d_.linear() = cmds_->getBaseRotationReference();
        rotTorpy(tmp_affine3d_.linear().transpose(),des_base_rpy_);
        tmp_affine3d_.translation().z() = cmds_->getBaseHeight();
        id_prob_->_waistRPY->setReference(tmp_affine3d_);

        id_prob_->_waistZ->setReference(tmp_affine3d_);

        if(inertia_compensation_active_)
        {
            Mi_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,M_.rows()-FLOATING_BASE_DOFS,M_.cols()-FLOATING_BASE_DOFS)
                    = M_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,M_.rows()-FLOATING_BASE_DOFS,M_.cols()-FLOATING_BASE_DOFS).inverse();
        }

        for(unsigned int i = 0; i<feet_names_.size(); i++)
        {
            // Update the reference for the feet tasks, this is only used for visualization pourposes
            id_prob_->_feet[feet_names_[i]]->setReference(gait_generator_->getReference(feet_names_[i]),gait_generator_->getReferenceDot(feet_names_[i]));

            // FIXME I should spline the wrench limits to load correctly the legs in stance and unload the swinging leg
            // Set the wrench limits to enstablish the contacts
            if(gait_generator_->isSwinging(feet_names_[i]))
            {
                id_prob_->_feet[feet_names_[i]]->setActive(false);
                id_prob_->_wrenches_lims->getWrenchLimits(feet_names_[i])->releaseContact(true);
                ROS_DEBUG_STREAM("Swinging: "<< feet_names_[i]);
                id_prob_->_postural_feet_swing[feet_names_[i]]->setActive(true);
                id_prob_->_postural_feet_stance[feet_names_[i]]->setActive(false);

                if(inertia_compensation_active_)
                {
                    Kp_postural_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Mi_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) * Kp_swing_leg_;
                    Kd_postural_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Mi_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) * Kd_swing_leg_;
                }
                else
                {
                    Kp_postural_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Kp_swing_leg_;
                    Kd_postural_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Kd_swing_leg_;
                }
            }
            else
            {
                Kp_postural_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Kp_stance_leg_;
                Kd_postural_.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Kd_stance_leg_;

                id_prob_->_feet[feet_names_[i]]->setActive(true);
                id_prob_->_wrenches_lims->getWrenchLimits(feet_names_[i])->releaseContact(false);
                ROS_DEBUG_STREAM("Stance: "<< feet_names_[i]);
                id_prob_->_postural_feet_swing[feet_names_[i]]->setActive(false);
                id_prob_->_postural_feet_stance[feet_names_[i]]->setActive(true);
            }
        }

        id_prob_->_postural->setGains(Kp_postural_,Kd_postural_);

        // Set the desired base height
        kin_->setDesiredBaseHeight(cmds_->getBaseHeight());

        // Update the desired joint positions from the ik and set that to the postural
        // task
        kin_->update(period.toSec(),joint_positions_);

        des_joint_positions_ = kin_->getDesiredJointPositions();
        des_joint_velocities_ = kin_->getDesiredJointVelocities();

        id_prob_->_postural->setReference(des_joint_positions_,des_joint_velocities_);

        // Solver Update
        id_prob_->update();

        // Get the solver solution
        x_ = des_joint_efforts_solver_; // Store the old desired efforts, to apply in case the solver gets mad
        if(!id_prob_->solve(x_))
        {
            ROS_ERROR("OpenSoT::IDProblem: unable to solve");
        }
        else
            des_joint_efforts_solver_ = x_;

        id_prob_->getGroundReactionForces(des_contact_forces_);

        pid_active_ = false;

    }
    else // Use a position PID controller
    {
        pid_active_ = true;
    }

    if(pid_active_)
    {
        des_joint_positions_ = kin_->getJointHomePositions();
        des_joint_efforts_solver_.fill(0.0);
        pid_scale_ = 1.0;
    }
    else
        pid_scale_ = 0.0;

    for (unsigned int i = 0; i < des_joint_p_gain_.size(); i++)
    {
        des_joint_p_gain_[i] = pid_scale_ * joint_p_gain_[i];
        des_joint_i_gain_[i] = pid_scale_ * joint_i_gain_[i];
        des_joint_d_gain_[i] = pid_scale_ * joint_d_gain_[i];
    }

    // Write to the hardware interface
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {

        pids_[i].setGains(des_joint_p_gain_[i],des_joint_i_gain_[i],des_joint_d_gain_[i],0,0);

        des_joint_efforts_pids_(i) = pids_[i].computeCommand(des_joint_positions_(i+FLOATING_BASE_DOFS)-joint_positions_(i+FLOATING_BASE_DOFS),
                                                             -joint_velocities_(i+FLOATING_BASE_DOFS),
                                                             period);

        des_joint_efforts_(i) = (1.0 - pid_scale_) * des_joint_efforts_solver_(i+FLOATING_BASE_DOFS) + pid_scale_ * des_joint_efforts_pids_(i);

        joint_states_[i].setCommand(des_joint_efforts_(i));
    }

    // Publish
    publish(time,period);
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
        base_pose = state_estimator_->getFloatingBasePose(); //FIXME Is it thread safe?

        // Do the inverse of it
        world_pose = base_pose.inverse();
        position = world_pose.translation();
        quaternion = world_pose.linear();
        quaternion.normalize();

        // Create the tf transform between /ci/base_link and /ci/world_odom (world)
        transform.setOrigin(tf::Vector3(position(0),position(1),position(2)));

        q.setX(quaternion.x());
        q.setY(quaternion.y());
        q.setZ(quaternion.z());
        q.setW(quaternion.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/base_link" , "/world" ));

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

void Controller::initPublishers(const ros::NodeHandle& root_nh, const ros::NodeHandle& controller_nh)
{
    // Create the realtime publishers
    ci_joint_states_rt_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "ci/joint_states", 4));
    ci_joint_states_rt_pub_->msg_.name.resize(_dof_names.size());
    ci_joint_states_rt_pub_->msg_.position.resize(_dof_names.size());
    ci_joint_states_rt_pub_->msg_.velocity.resize(_dof_names.size());
    ci_joint_states_rt_pub_->msg_.effort.resize(_dof_names.size());
    for (unsigned int i = 0; i < _dof_names.size(); i++)
        ci_joint_states_rt_pub_->msg_.name[i] = _dof_names[i];

    contact_forces_pub_.reset(new realtime_tools::RealtimePublisher<wb_controller::ContactForces>(controller_nh, "contact_forces", 4));
    contact_forces_pub_->msg_.header.frame_id = "world"; //FIXME
    contact_forces_pub_->msg_.name.resize(4);
    contact_forces_pub_->msg_.contact.resize(4);
    contact_forces_pub_->msg_.contact_positions.resize(4);
    contact_forces_pub_->msg_.contact_forces.resize(4);
    contact_forces_pub_->msg_.des_contact_forces.resize(4);
}

void Controller::publish(const ros::Time& time, const ros::Duration& /*period*/)
{
    // FIXME it should not be there but for the moment I need it here because of the twist reset in the update of the solver:
    if(id_prob_)
        id_prob_->publish(time);

    if(contact_forces_pub_.get() && contact_forces_pub_->trylock())
    {
        for(unsigned int i=0; i <feet_names_.size(); i++)
        {
            contact_forces_pub_->msg_.name[i] = feet_names_[i];
            contact_forces_pub_->msg_.contact[i] = state_estimator_->getContacts()[i];
            contact_forces_pub_->msg_.contact_positions[i].x = state_estimator_->getFeetPositionInWorld()[i](0);
            contact_forces_pub_->msg_.contact_positions[i].y = state_estimator_->getFeetPositionInWorld()[i](1);
            contact_forces_pub_->msg_.contact_positions[i].z = state_estimator_->getFeetPositionInWorld()[i](2);

            contact_forces_pub_->msg_.contact_forces[i].force.x = state_estimator_->getContactForces()[i](0);
            contact_forces_pub_->msg_.contact_forces[i].force.y = state_estimator_->getContactForces()[i](1);
            contact_forces_pub_->msg_.contact_forces[i].force.z = state_estimator_->getContactForces()[i](2);

            contact_forces_pub_->msg_.des_contact_forces[i].force.x = des_contact_forces_.segment(6*i,3)(0);
            contact_forces_pub_->msg_.des_contact_forces[i].force.y = des_contact_forces_.segment(6*i,3)(1);
            contact_forces_pub_->msg_.des_contact_forces[i].force.z = des_contact_forces_.segment(6*i,3)(2);
        }
        contact_forces_pub_->msg_.header.stamp = time;
        contact_forces_pub_->unlockAndPublish();
    }

    if(ci_joint_states_rt_pub_.get() && ci_joint_states_rt_pub_->trylock())
    {
        xbot_model_->getJointPosition(joint_positions_xbot_);
        xbot_model_->getJointVelocity(joint_velocities_xbot_);

        for(unsigned int i = 0; i < joint_positions_.size(); i++)
        {
            ci_joint_states_rt_pub_->msg_.position[i]  = joint_positions_xbot_(i);
            ci_joint_states_rt_pub_->msg_.velocity[i]  = joint_velocities_xbot_(i);
            ci_joint_states_rt_pub_->msg_.effort[i]    = des_joint_efforts_solver_(i);
        }
        ci_joint_states_rt_pub_->msg_.header.stamp = time;
        ci_joint_states_rt_pub_->unlockAndPublish();
    }

    // Logger publishing
    RtLogger::getLogger().publish(time);
}

void Controller::stopping(const ros::Time& /*time*/)
{
    ROS_DEBUG_NAMED(CLASS_NAME,"Stopping the Controller");

    stopping_ = true;
    odom_publisher_thread_->join();

    ROS_DEBUG_NAMED(CLASS_NAME,"Stopping Controller Completed");
}

GaitGenerator* Controller::getGaitGenerator() const
{
    return gait_generator_.get();
}

StateEstimator* Controller::getStateEstimator() const
{
    return state_estimator_.get();
}

FootholdsPlanner* Controller::getFootholdsPlanner() const
{
    return cmds_.get();
}

LegsKinematics* Controller::getLegsKinematics() const
{
    return kin_.get();
}

} //namespace
