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
#include <wb_controller/ros_wrappers/controller.h>
#include <wb_controller/devices/joy.h>
#include <wb_controller/devices/twist.h>

using namespace XBot;
using namespace Cartesian;
using namespace rt_logger;

namespace wb_controller {

std::vector<std::string> _dof_names = {}; // To be loaded from the robot model
std::vector<std::string> _cartesian_names = {"x","y","z","roll","pitch","yaw"}; // This is our standard cartesian dofs order
std::vector<std::string> _xyz = {"x","y","z"};
std::vector<std::string> _rpy = {"roll","pitch","yaw"};
std::vector<std::string> _joints_prefix = {"haa","hfe","kfe"};
std::vector<std::string> _legs_prefix = {"lf","lh","rf","rh"};
double _period = 0.001;

Controller::Controller()
    :stopping_(false)
    ,mode_(WALKING)
    ,posture_(DOWN)
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
    hardware_interface::ContactSwitchSensorInterface* cs_hw = robot_hw->get<hardware_interface::ContactSwitchSensorInterface>();

    // Hardware interfaces checks
    if(!jt_hw)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::EffortJointInterface not found");
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

    robot_model_.reset(createRobotModel(root_nh));
    joint_names_ = robot_model_->getJointNames();

    if(!cs_hw)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::ContactSwitchSensorInterface not found");
        return false;
    }
    else
    {
        contact_sensors_["lf_foot"] = cs_hw->getHandle("lf_foot_contact_sensor"); // FIXME Hardcoded
        contact_sensors_["rf_foot"] = cs_hw->getHandle("rf_foot_contact_sensor"); // FIXME Hardcoded
        contact_sensors_["lh_foot"] = cs_hw->getHandle("lh_foot_contact_sensor"); // FIXME Hardcoded
        contact_sensors_["rh_foot"] = cs_hw->getHandle("rh_foot_contact_sensor"); // FIXME Hardcoded
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
        imu_name_ = "trunk_imu"; //FIXME note the hardcoded name...
        ROS_DEBUG_STREAM("Found imu sensor: "<<imu_name_);
        imu_sensor_ = imu_hw->getHandle(imu_name_);
    }
    catch(...)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"Error loading imu_sensor_");
        return false;
    }

    if(!root_nh.getParam("/task_period",period_)) // Get the initial task period
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No task period given in namespace /");
        return false;
    }
    _period = period_;

    if(!root_nh.getParam("/internal_wrench",use_contact_sensors_)) // Use the contact sensors
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact estimation");
    else
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact sensors");

    // Resize the variables
    joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_velocities_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_velocities_filt_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_accellerations_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_efforts_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_velocities_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_efforts_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_efforts_solver_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_joint_efforts_impedance_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    des_contact_forces_.resize(robot_model_->getContactNames().size(),Eigen::Vector6d::Zero());

    // Initializations
    joint_positions_.fill(0.0);
    joint_velocities_.fill(0.0);
    joint_velocities_filt_.fill(0.0);
    joint_accellerations_.fill(0.0);
    joint_efforts_.fill(0.0);
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.fill(0.0);
    des_joint_efforts_impedance_.fill(0.0);
    des_joint_efforts_solver_.fill(0.0);
    imu_orientation_.normalize();

    gait_generator_ = std::make_shared<GaitGenerator>(robot_model_->getFootNames(),Gait::TROT);
    foot_holds_planner_ = std::make_shared<FootholdsPlanner>(gait_generator_,robot_model_);
    state_estimator_    = std::make_shared<StateEstimator>(gait_generator_,robot_model_);

    legs_impedance_     = std::make_shared<LegsImpedance>(gait_generator_,robot_model_);
    legs_impedance_->startInertiaCompensation(true);

    terrain_estimator_ = std::make_shared<TerrainEstimator>(state_estimator_,foot_holds_planner_,robot_model_);
    terrain_estimator_->setMaxRoll(M_PI);
    terrain_estimator_->setMinRoll(-M_PI);
    terrain_estimator_->setMaxPitch(M_PI);
    terrain_estimator_->setMinPitch(-M_PI);

    com_planner_ = std::make_shared<ComPlanner>(robot_model_,foot_holds_planner_,terrain_estimator_);
    id_prob_ = std::make_unique<IDProblem>(nh_,robot_model_,period_);

    solver_failures_cnt_   = std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_)));
    contact_failures_cnt_  = std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_)));
    for(unsigned int i=0;i<joint_velocities_.size();i++)
      velocity_lims_failures_cnt_.push_back(std::make_shared<Counter>(static_cast<int>(std::ceil(0.1 / period_))));

    ramp_up_   = std::make_shared<Ramp>(5.0,Ramp::UP);
    ramp_down_ = std::make_shared<Ramp>(5.0,Ramp::DOWN);

    std::string input_device = "ps3";
    root_nh.getParam("/input_device",input_device);
    if(input_device == "ps3")
    {
        device_handler_ = std::make_shared<Ps3JoyHandler>(controller_nh,this);
        ROS_INFO_NAMED(CLASS_NAME,"Use PS3 controller device");
    }
    else if(input_device == "xbox")
    {
        device_handler_ = std::make_shared<XboxJoyHandler>(controller_nh,this);
        ROS_INFO_NAMED(CLASS_NAME,"Use XBOX controller device");
    }
    else if(input_device == "twist")
    {
        device_handler_ = std::make_shared<TwistHandler>(controller_nh,this);
        ROS_INFO_NAMED(CLASS_NAME,"Use ROS::twist input device");
    }
    else if(input_device == "keyboard")
    {
        device_handler_ = std::make_shared<TwistHandler>(controller_nh,this);
        ROS_INFO_NAMED(CLASS_NAME,"Use ROS::twist input device");
    }
    else
    {
        ROS_ERROR_NAMED(CLASS_NAME,"Wrong input_device");
        return false;
    }

    // Spawn the odom publisher thread
    odom_publisher_thread_.reset(new std::thread(&Controller::odomPublisher,this));

    RtLogger::getLogger().addPublisher(CLASS_NAME+"/imu_gyroscope",imu_gyroscope_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/imu_gyroscope_filt",imu_gyroscope_filt_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_velocities",des_joint_velocities_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/joint_velocities",joint_velocities_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/joint_velocities_filt",joint_velocities_filt_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_efforts_solver",des_joint_efforts_solver_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_efforts_impedance",des_joint_efforts_impedance_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_efforts",des_joint_efforts_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/joint_efforts",joint_efforts_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_base_rpy",des_base_rpy_);
    RtLogger::getLogger().addPublisher(CLASS_NAME+"/period",period_);

    ros_wrapper_ = std::make_shared<ControllerRosWrapper>(root_nh,controller_nh,this);

    return true;
}

bool Controller::setFrictionConesMu(const double& mu)
{

    if(id_prob_)
    {
        if(mu>=0.0 && mu<=1.0)
        {
            id_prob_->setFrictionConesMu(mu);
            ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set mu to: "<<mu);
        }
        else
        {
            ROS_WARN_NAMED(CLASS_NAME,"Mu has to be between 0 and 1!");
            return false;
        }
    }
    else
    {
        ROS_WARN_NAMED(CLASS_NAME,"Did you press start?");
        return false;
    }
    return true;
}


void Controller::setCutoffFreqQdot(const double &hz)
{
    if(hz >= 0.0)
    {
        qdot_filter_.setOmega(2.0*M_PI*hz);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set cutoff frequency for qdot filter at "<< hz);
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Cutoff frequency has to be >= 0.0 [Hz]!");
}

void Controller::setCutoffFreqGyro(const double &hz)
{
    if(hz >= 0.0)
    {
        imu_gyroscope_filter_.setOmega(2.0*M_PI*hz);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set cutoff frequency for imu gyroscope filter at "<< hz);
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Cutoff frequency has to be >= 0.0 [Hz]!");
}

bool Controller::setSwingFrequency(const double& swing_frequency)
{
    const std::vector<std::string>& foot_names = robot_model_->getFootNames();
    if(gait_generator_)
    {
        for(unsigned int i=0; i < foot_names.size(); i++)
            gait_generator_->setSwingFrequency(foot_names[i],swing_frequency);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set swing frequency to "<< swing_frequency);
    }
    else
    {
        ROS_WARN_NAMED(CLASS_NAME,"gait_generator not initialized yet.");
        return false;
    }
    return true;
}

bool Controller::selectControlMode(const std::string& mode)
{
  if(mode == "WALKING")
    mode_ = Controller::mode_t::WALKING;
  else if(mode == "MANIPULATION")
    mode_ = Controller::mode_t::MANIPULATION;
  else
  {
    ROS_ERROR_NAMED(CLASS_NAME,"Wrong mode!");
    return false;
  }
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Selected mode "<< mode);

  return true;
}

void Controller::switchControlMode()
{
  if(mode_ == Controller::mode_t::WALKING)
    mode_ = Controller::mode_t::MANIPULATION;
  else
    mode_ = Controller::mode_t::WALKING;
}

bool Controller::selectPosture(const std::string& posture)
{
  if(posture == "UP")
  {
    mode_ = Controller::posture_t::UP;
  }
  else if(posture == "DOWN")
    mode_ = Controller::posture_t::DOWN;
  else
  {
    ROS_ERROR_NAMED(CLASS_NAME,"Wrong posture!");
    return false;
  }
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Selected posture "<< posture);
  return true;
}

void Controller::switchPosture()
{
  if(posture_ == Controller::posture_t::UP)
    posture_ = Controller::posture_t::DOWN;
  else
    posture_ = Controller::posture_t::UP;
}

void Controller::standUp(bool stand_up)
{
  if(stand_up)
    posture_ = Controller::posture_t::UP;
  else
    posture_ = Controller::posture_t::DOWN;
}

bool Controller::selectGait(const string& gait)
{
  if(gait_generator_)
  {
    if(gait == "TROT")
      gait_generator_->setGaitType(Gait::gait_t::TROT);
    else if(gait == "CRAWL")
      gait_generator_->setGaitType(Gait::gait_t::CRAWL);
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Wrong gait!");
      return false;
    }
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Selected gait "<< gait);
  }
  return true;
}

void Controller::switchGait()
{
  if(gait_generator_)
  {
    gait_generator_->switchGait();
  }
  else
    ROS_WARN_NAMED(CLASS_NAME,"GaitGenerator not ready yet!");
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

void Controller::updateStateEstimator(const double &dt)
{
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
        ground_truth_orientation_.w() = ground_truth_.getOrientation()[0];
        ground_truth_orientation_.x() = ground_truth_.getOrientation()[1];
        ground_truth_orientation_.y() = ground_truth_.getOrientation()[2];
        ground_truth_orientation_.z() = ground_truth_.getOrientation()[3];
        state_estimator_->setGroundTruthBaseOrientation(ground_truth_orientation_);
        state_estimator_->setGroundTruthBaseAngularVelocity(Eigen::Map<const Eigen::Vector3d>(ground_truth_.getAngularVelocity()));
    }
    else
    {
        state_estimator_->setImuOrientation(imu_orientation_);
        state_estimator_->setImuGyroscope(imu_gyroscope_filt_);
    }

    const std::vector<std::string>& foot_names = robot_model_->getFootNames();
    if(use_contact_sensors_)
        for(unsigned int i = 0; i<foot_names.size(); i++)
        {
            tmp_vector3d_[0] = contact_sensors_[foot_names[i]].getForce()[0];
            tmp_vector3d_[1] = contact_sensors_[foot_names[i]].getForce()[1];
            tmp_vector3d_[2] = contact_sensors_[foot_names[i]].getForce()[2];
            state_estimator_->setContactForces(foot_names[i],tmp_vector3d_);
        }

    state_estimator_->update(dt);

}

void Controller::updateStateMachine(const double &dt)
{
    double desired_height;
    double current_height = robot_model_->getCurrentHeight();
    unsigned int current_state = robot_model_->getState();
    switch(current_state)
    {
      case(QuadrupedRobot::IDLE):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: IDLE");
        if(posture_ == Controller::posture_t::UP)
          robot_model_->setState(QuadrupedRobot::INIT);
        break;

      case(QuadrupedRobot::INIT):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: INIT");
        init();
        robot_model_->setState(QuadrupedRobot::STANDING_UP);
        break;

      case(QuadrupedRobot::STANDING_UP):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: STANDING_UP");
        updateComponents(dt);
        desired_height = ramp_up_->update(dt) * robot_model_->getStandUpHeight();
        tmp_matrix3d_.setIdentity();
        tmp_vector3d_.setZero(); // com position
        tmp_vector3d_ << com_planner_->getComPosition().x(), com_planner_->getComPosition().y(), desired_height;
        tmp_vector3d_1_.setZero(); // com velocity
        tmp_vector3d_1_.z() = foot_holds_planner_->getLinearVelocityCmd();
        updateBaseReferences(tmp_vector3d_,tmp_vector3d_1_,tmp_matrix3d_);
        if(!updateSolver(dt) || !performSafetyChecks())
        {
          robot_model_->setState(QuadrupedRobot::IMPEDANCE);
          break;
        }
        if(current_height >= robot_model_->getStandUpHeight())
        {
          ramp_up_->reset();
          if(mode_ == Controller::mode_t::WALKING)
            robot_model_->setState(QuadrupedRobot::WALKING);
          else if (mode_ == Controller::mode_t::MANIPULATION)
            robot_model_->setState(QuadrupedRobot::MANIPULATION);
          else
            robot_model_->setState(QuadrupedRobot::WALKING);
          foot_holds_planner_->reset();
        }
        break;

      case(QuadrupedRobot::WALKING):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: WALKING");
        updateComponents(dt);
        updateBaseReferences(com_planner_->getComPosition(),com_planner_->getComVelocity(),foot_holds_planner_->getBaseRotationReference());
        if(!updateSolver(dt) || !performSafetyChecks())
        {
          robot_model_->setState(QuadrupedRobot::IMPEDANCE);
          break;
        }
        if(mode_ == Controller::mode_t::MANIPULATION)
          robot_model_->setState(QuadrupedRobot::MANIPULATION);
        if(posture_ == Controller::posture_t::DOWN)
        {
          stand_down_starting_height_ = current_height;
          robot_model_->setState(QuadrupedRobot::STANDING_DOWN);
        }
        break;

      case(QuadrupedRobot::MANIPULATION):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: MANIPULATION");
        updateComponents(dt);
        updateBaseReferences(com_planner_->getComPosition(),com_planner_->getComVelocity(),foot_holds_planner_->getBaseRotationReference());
        if(!updateSolver(dt) || !performSafetyChecks())
        {
          robot_model_->setState(QuadrupedRobot::IMPEDANCE);
          break;
        }
        if(mode_ == Controller::mode_t::WALKING)
          robot_model_->setState(QuadrupedRobot::WALKING);
        if(posture_ == Controller::posture_t::DOWN)
        {
          stand_down_starting_height_ = current_height;
          robot_model_->setState(QuadrupedRobot::STANDING_DOWN);
        }
        break;

      case(QuadrupedRobot::STANDING_DOWN):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: STANDING_DOWN");
        updateComponents(dt);
        desired_height = ramp_down_->update(dt) * stand_down_starting_height_;
        tmp_matrix3d_.setIdentity();
        tmp_vector3d_.setZero(); // com position
        tmp_vector3d_ << com_planner_->getComPosition().x(), com_planner_->getComPosition().y(), desired_height;
        tmp_vector3d_1_.setZero(); // com velocity
        tmp_vector3d_1_.z() = -foot_holds_planner_->getLinearVelocityCmd();
        updateBaseReferences(tmp_vector3d_,tmp_vector3d_1_,tmp_matrix3d_);
        if(!updateSolver(dt) || !performSafetyChecks())
        {
          robot_model_->setState(QuadrupedRobot::IMPEDANCE);
          break;
        }
        if(current_height <= robot_model_->getStandDownHeight())
        {
          ramp_down_->reset();
          robot_model_->setState(QuadrupedRobot::IDLE);
        }
        break;

      case(QuadrupedRobot::IMPEDANCE):
        ROS_INFO_ONCE_NAMED(CLASS_NAME,"State: IMPEDANCE");
        updateImpedance(dt);
        if(current_height <= robot_model_->getStandDownHeight())
        {
          posture_ = Controller::posture_t::DOWN;
          robot_model_->setState(QuadrupedRobot::IDLE);
        }
        break;
    };
}
void Controller::init()
{
    // State estimator
    // Be sure to start the solver and the contact estimation when the robot is grounded.
    state_estimator_->resetGyroscopeIntegration();
    state_estimator_->startContactComputation();
    state_estimator_->startHapticContactLoop();
    // Footholds planner with gait generator
    foot_holds_planner_->reset();
    foot_holds_planner_->setBasePosition(state_estimator_->getFloatingBasePosition());
    foot_holds_planner_->setDefaultBasePosition(Eigen::Vector3d(0.0,0.0,robot_model_->getStandUpHeight()));
    foot_holds_planner_->setBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
    foot_holds_planner_->setDefaultBaseOrientation(Eigen::Vector3d(0.0,0.0,0.0));
    foot_holds_planner_->initializeFeetPosition();
    // Filters
    imu_gyroscope_filter_.setTimeStep(period_);
    qdot_filter_.setTimeStep(period_);
    // Counters for safety checks
    contact_failures_cnt_->reset();
    solver_failures_cnt_->reset();
    for(unsigned int i=0;i<velocity_lims_failures_cnt_.size();i++)
      velocity_lims_failures_cnt_[i]->reset();
    // Control torques
    des_joint_efforts_.fill(0.0);
    des_joint_efforts_solver_.fill(0.0);
    des_joint_efforts_impedance_.fill(0.0);
}

void Controller::updateComponents(const double &dt)
{
  // Update the footholds planner
  foot_holds_planner_->update(dt);
  // Update the terrain estimator
  terrain_estimator_->computeTerrainEstimation(dt);
  // Update the CoM position and velocity reference
  com_planner_->update();
  // Transform the desired base rotation into RPY for visualization
  rotTorpy(foot_holds_planner_->getBaseRotationReference().transpose(),des_base_rpy_);
}

void Controller::updateBaseReferences(const Eigen::Vector3d &com_pos_ref, const Eigen::Vector3d &com_vel_ref, const Eigen::Matrix3d &orientation_ref)
{
  // Set the pose reference for the waist
  id_prob_->setWaistReference(orientation_ref,com_pos_ref.z());
  // Set the velocity and position reference for the CoM in the solver
  id_prob_->setComReference(com_pos_ref,com_vel_ref);
}

bool Controller::performSafetyChecks()
{

  bool ok = true;

  // Check if we have at least one contact
  auto contacts = state_estimator_->getContacts();
  bool contact = false;
  for (auto& tmp_map : contacts)
    contact = contact || tmp_map.second;
  if(!contact) // && state_estimator_->getFloatingBasePosition().z() > 0.3 * robot_model_->getStandUpHeight()
    contact_failures_cnt_->increase();
  else
    contact_failures_cnt_->reset();
  if(contact_failures_cnt_->upperLimitReached())
  {
    ok = false;
    ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Lost contacts!");
  }

  // Check if the current joint velocities are valid otherwise set robot state to anomaly
  std::vector<bool>&& checks = robot_model_->checkJointVelocities(joint_velocities_);
  for(unsigned int i=0;i<checks.size();i++)
  {
    if(checks[i])
      velocity_lims_failures_cnt_[i]->increase();
    else
      velocity_lims_failures_cnt_[i]->reset();
    if(velocity_lims_failures_cnt_[i]->upperLimitReached())
    {
      ok = false;
      auto names = robot_model_->getEnabledJointNames();
      ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Reached joint velocity limit "<<names[i]);
    }
  }

  return ok;
}

void Controller::updateImpedance(const double& /*dt*/)
{
  legs_impedance_->update();
  des_joint_efforts_impedance_ = - legs_impedance_->getKd() * joint_velocities_;
}

bool Controller::updateSolver(const double &/*dt*/)
{
  // Rotate the friction cones based on the terrain orientation
  id_prob_->setFrictionConesR(terrain_estimator_->getTerrainOrientationWorld().transpose());

  // Set the references to the feet based on their current state: Stance/Swing
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
      id_prob_->setFootReference(foot_names[i],gait_generator_->getReference(foot_names[i]),gait_generator_->getReferenceDot(foot_names[i]),
                                 WORLD_FRAME_NAME);
      if(gait_generator_->isSwinging(foot_names[i]))
      {
          id_prob_->swingWithFoot(foot_names[i]);
          ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Swinging: "<< foot_names[i]);
      }
      else
      {
          id_prob_->stanceWithFoot(foot_names[i]);
          ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Stance: "<< foot_names[i]);
      }
  }
  // Get the solver solution
  if(!id_prob_->solve(des_joint_efforts_solver_))
  {
    ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Failed to solve!");
    return false;
  }
  else
    return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // Reset control values
    des_joint_efforts_impedance_.fill(0.0);
    des_joint_efforts_solver_.fill(0.0);
    des_joint_efforts_.fill(0.0);

    period_ = period.toSec();

    // Read joint values from the hardware interface
    readJoints();

    // Read IMU values from the hardware interface
    readImu();

    // Update state estimator
    updateStateEstimator(period_);

    // Update state machine
    updateStateMachine(period_);

    // Desired joint efforts
    des_joint_efforts_ = des_joint_efforts_solver_ + des_joint_efforts_impedance_;

    // Saturate desired joint efforts
    robot_model_->clampJointEfforts(des_joint_efforts_);

    // Write to the hardware interface
    for (unsigned int i = 0; i < joint_states_.size(); i++)
      joint_states_[i].setCommand(des_joint_efforts_(i+FLOATING_BASE_DOFS));

    // Publish
    ros_wrapper_->publish(time);
    RtLogger::getLogger().publish(time);
}

void Controller::odomPublisher()
{
    ROS_INFO_NAMED(CLASS_NAME,"Start the odomPublisher");

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
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/"+robot_model_->getBaseLinkName() , "/world" ));

        // Create the tf transform between /ci/base_link and /base_link
        transform.setOrigin(tf::Vector3(0,0,0));
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/"+robot_model_->getBaseLinkName(), "/"+robot_model_->getBaseLinkName()));

        std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );
    }
    ROS_INFO_NAMED(CLASS_NAME,"Stop the odomPublisher");
}

void Controller::stopping(const ros::Time& /*time*/)
{
    ROS_DEBUG_NAMED(CLASS_NAME,"Stopping the Controller");

    stopping_ = true;
    odom_publisher_thread_->join();

    ROS_DEBUG_NAMED(CLASS_NAME,"Stopping Controller Completed");
}

IDProblem* Controller::getIDProblem() const
{
    return id_prob_.get();
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
    return foot_holds_planner_.get();
}

TerrainEstimator* Controller::getTerrainEstimator() const
{
    return terrain_estimator_.get();
}

LegsImpedance* Controller::getLegsImpedance() const
{
    return legs_impedance_.get();
}

QuadrupedRobot* Controller::getRobotModel() const
{
    return robot_model_.get();
}

std::vector<Eigen::Vector6d>& Controller::getDesiredContactForces()
{
    if(id_prob_)
        des_contact_forces_ = id_prob_->getContactWrenches();
    return des_contact_forces_;
}

} //namespace
