/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief This file contains the constructor, destructor, init, stopping and other facilities for the
 * WoLF controller.
 */

#include <wolf_controller/controller.h>
#include <wolf_controller/ros_wrappers/controller.h>
#include <wolf_controller/devices/joy.h>
#include <wolf_controller/devices/twist.h>
#include <wolf_controller/devices/keyboard.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace XBot;
using namespace Cartesian;
using namespace rt_logger;

namespace wolf_controller {

std::vector<std::string> _dof_names = {}; // To be loaded from the robot model
std::vector<std::string> _cartesian_names = {"x","y","z","roll","pitch","yaw"}; // This is our standard cartesian dofs order
std::vector<std::string> _xyz = {"x","y","z"};
std::vector<std::string> _rpy = {"roll","pitch","yaw"};
std::vector<std::string> _joints_prefix = {"haa","hfe","kfe"};
std::vector<std::string> _legs_prefix = {"lf","lh","rf","rh"};
double _period = 0.001;
std::string _robot_name = "";

Controller::Controller()
    :MultiInterfaceController<hardware_interface::EffortJointInterface,
                              hardware_interface::ImuSensorInterface,
                              hardware_interface::GroundTruthInterface,
                              hardware_interface::ContactSwitchSensorInterface> (true) // allow_optional_interfaces = true
    ,stopping_(false)
    ,mode_(WALKING)
    ,previous_mode_(WALKING)
    ,posture_(DOWN)
{
   XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::HIGH);
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
    root_nh_ = root_nh;

    assert(robot_hw);

    robot_model_.reset(createRobotModel(root_nh));
    joint_names_ = robot_model_->getJointNames();

    if(!root_nh.getParam("/task_period",period_)) // Get the initial task period
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No task period given in namespace /");
        return false;
    }
    _period = period_;

    if(!root_nh.getParam("/robot_name",robot_name_)) // Get the robot name
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No robot name given in namespace /");
        return false;
    }
    _robot_name = robot_name_;

    hardware_interface::EffortJointInterface* jt_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
    hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
    hardware_interface::GroundTruthInterface* gt_hw = robot_hw->get<hardware_interface::GroundTruthInterface>();
    hardware_interface::ContactSwitchSensorInterface* cs_hw = robot_hw->get<hardware_interface::ContactSwitchSensorInterface>();

    // Hardware interfaces: Joints
    if(!jt_hw)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::EffortJointInterface not found");
        return false;
    }
    else
    {
      // Setting up joint handles:
      for (unsigned int i = 0; i < joint_names_.size(); i++)
      {
          // Getting joint state handle
          try
          {
              ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Found joint: "<<joint_names_[i]);
              joint_states_.push_back(jt_hw->getHandle(joint_names_[i])); // FIXME
          }
          catch(...)
          {
              ROS_ERROR_NAMED(CLASS_NAME,"Error loading the joint handles");
              return false;
          }
      }
      assert(joint_states_.size()>0);
    }

    if(!imu_hw)
    {
        ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::ImuSensorInterface not found");
        return false;
    }
    else
    {
        try
        {
            std::string imu_sensor_name;
            if(controller_nh.getParam("imu_sensor_name",imu_sensor_name))
              imu_sensor_ = imu_hw->getHandle(imu_sensor_name); // Take the selected imu sensor
            else
              imu_sensor_ = imu_hw->getHandle(imu_hw->getNames()[0]); // Take the first imu sensor
        }
        catch(...)
        {
            ROS_ERROR_NAMED(CLASS_NAME,"Error loading the imu handler");
            return false;
        }
    }

    if(!gt_hw)
        ROS_WARN_NAMED(CLASS_NAME,"hardware_interface::GroundTruthInterface not found");
    else
        ground_truth_ = gt_hw->getHandle(gt_hw->getNames()[0]);

    use_contact_sensors_ = false;
    if(controller_nh.getParam("use_contact_sensors",use_contact_sensors_))
    {
      if(!cs_hw)
      {
          ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::ContactSwitchSensorInterface not found");
          return false;
      }
      else
      {
          auto contact_sensor_names = cs_hw->getNames();
          if(contact_sensor_names.size() != N_LEGS)
          {
            ROS_ERROR_NAMED(CLASS_NAME,"Wrong number of contact sensors! only (4) are supported. Did you specify the contacts in the SRDF file?");
            return false;
          }
          auto foot_names = robot_model_->getFootNames();
          contact_sensor_names = sortByLegPrefix(contact_sensor_names);
          for(unsigned int i=0;i<contact_sensor_names.size();i++)
            contact_sensors_[foot_names[i]] = cs_hw->getHandle(contact_sensor_names[i]);
      }
    }
    if(!use_contact_sensors_) // Use the contact sensors
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact estimation");
    else
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact sensors");

    // Resize the variables
    joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
    joint_positions_init_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
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
    imu_gyroscope_.fill(0.0);
    imu_accelerometer_.fill(0.0);
    imu_gyroscope_filt_.fill(0.0);
    imu_accelerometer_filt_.fill(0.0);

    gait_generator_ = std::make_shared<GaitGenerator>(robot_model_->getFootNames(),Gait::TROT);
    foot_holds_planner_ = std::make_shared<FootholdsPlanner>(gait_generator_,robot_model_);
    state_estimator_    = std::make_shared<StateEstimator>(gait_generator_,robot_model_);

    impedance_     = std::make_shared<Impedance>(gait_generator_,robot_model_);
    impedance_->startInertiaCompensation(false);

    terrain_estimator_ = std::make_shared<TerrainEstimator>(state_estimator_,foot_holds_planner_,robot_model_);
    terrain_estimator_->setMaxRoll(M_PI);
    terrain_estimator_->setMinRoll(-M_PI);
    terrain_estimator_->setMaxPitch(M_PI);
    terrain_estimator_->setMinPitch(-M_PI);

    com_planner_ = std::make_shared<ComPlanner>(robot_model_,foot_holds_planner_,terrain_estimator_);
    id_prob_ = std::make_unique<IDProblem>(robot_model_);

    solver_failures_cnt_   = std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_)));
    contact_failures_cnt_  = std::make_shared<Counter>(static_cast<int>(std::ceil(0.5 / period_)));
    for(unsigned int i=0;i<joint_velocities_.size();i++)
      velocity_lims_failures_cnt_.push_back(std::make_shared<Counter>(static_cast<int>(std::ceil(0.1 / period_))));

    ramp_stand_up_    = std::make_shared<Ramp>(5.0,Ramp::UP);
    ramp_stand_down_  = std::make_shared<Ramp>(5.0,Ramp::DOWN);
    ramp_init_        = std::make_shared<Ramp>(3.0,Ramp::UP);

    previous_height_ = robot_model_->getStandUpHeight();

    std::string input_device = "ps3";
    root_nh.getParam("/input_device",input_device);
    if(input_device == "ps3")
        devices_.addDevice(DevicesHandler::priority_t::HIGH,std::make_shared<Ps3JoyHandler>(controller_nh,this)); // Ps3 joy
    else if(input_device == "xbox")
        devices_.addDevice(DevicesHandler::priority_t::HIGH,std::make_shared<XboxJoyHandler>(controller_nh,this)); // Xbox joy
    else if(input_device == "spacemouse")
        devices_.addDevice(DevicesHandler::priority_t::HIGH,std::make_shared<SpaceJoyHandler>(controller_nh,this)); // Space joy
    else if(input_device == "keyboard")
        devices_.addDevice(DevicesHandler::priority_t::HIGH,std::make_shared<KeyboardHandler>(controller_nh,this)); // Keyboard
    devices_.addDevice(DevicesHandler::priority_t::LOW,std::make_shared<TwistHandler>(controller_nh,this)); // Twist

    // HACK
    estimator_ = std::make_shared<wolf_estimation::BaseEstimator>(robot_model_->getUrdfString(),robot_model_->getSrdfString(),
                                                                  robot_model_->getFootNames(),robot_model_->getImuSensorName(),
                                                                  robot_model_->getBaseLinkName());

    // Spawn the odom publisher thread
    odom_publisher_thread_= std::make_shared<std::thread>(&Controller::odomPublisher,this);

    RtLogger::getLogger().addPublisher(TOPIC(imu_gyroscope)               ,imu_gyroscope_);
    RtLogger::getLogger().addPublisher(TOPIC(imu_gyroscope_filt)          ,imu_gyroscope_filt_);
    RtLogger::getLogger().addPublisher(TOPIC(des_joint_positions)         ,des_joint_positions_);
    RtLogger::getLogger().addPublisher(TOPIC(joint_positions)             ,joint_positions_);
    RtLogger::getLogger().addPublisher(TOPIC(des_joint_velocities)        ,des_joint_velocities_);
    RtLogger::getLogger().addPublisher(TOPIC(joint_velocities)            ,joint_velocities_);
    RtLogger::getLogger().addPublisher(TOPIC(joint_velocities_filt)       ,joint_velocities_filt_);
    RtLogger::getLogger().addPublisher(TOPIC(des_joint_efforts_solver)    ,des_joint_efforts_solver_);
    RtLogger::getLogger().addPublisher(TOPIC(des_joint_efforts_impedance) ,des_joint_efforts_impedance_);
    RtLogger::getLogger().addPublisher(TOPIC(des_joint_efforts)           ,des_joint_efforts_);
    RtLogger::getLogger().addPublisher(TOPIC(joint_efforts)               ,joint_efforts_);
    RtLogger::getLogger().addPublisher(TOPIC(period)                      ,period_);

    ros_wrapper_ = std::make_shared<ControllerRosWrapper>(root_nh,controller_nh,this);

    id_prob_->init(nh_,period_);

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

void Controller::setCutoffFreqGyroscope(const double &hz)
{
    if(hz >= 0.0)
    {
        imu_gyroscope_filter_.setOmega(2.0*M_PI*hz);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set cutoff frequency for imu gyroscope filter at "<< hz);
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Cutoff frequency has to be >= 0.0 [Hz]!");
}

void Controller::setCutoffFreqAccelerometer(const double &hz)
{
    if(hz >= 0.0)
    {
        imu_accelerometer_filter_.setOmega(2.0*M_PI*hz);
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set cutoff frequency for imu accelerometer filter at "<< hz);
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

bool Controller::setStepHeight(const double& step_height)
{
  if(foot_holds_planner_)
      foot_holds_planner_->setStepHeight(step_height);
  else
  {
      ROS_WARN_NAMED(CLASS_NAME,"foot_holds_planner not initialized yet.");
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
  else if(mode == "RESET")
    mode_ = Controller::mode_t::RESET;
  else
  {
    ROS_ERROR_NAMED(CLASS_NAME,"Wrong mode!");
    return false;
  }
  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Selected mode "<< mode);

  return true;
}

unsigned int Controller::getControlMode()
{
  return mode_;
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
    posture_ = Controller::posture_t::UP;
  }
  else if(posture == "DOWN")
    posture_ = Controller::posture_t::DOWN;
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

void Controller::resetBase()
{
  mode_ = Controller::mode_t::RESET;
}

void Controller::emergencyStop()
{
  robot_model_->setState(QuadrupedRobot::ANOMALY);
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
    imu_gyroscope_     = Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getAngularVelocity());
    imu_orientation_.w() = imu_sensor_.getOrientation()[0];
    imu_orientation_.x() = imu_sensor_.getOrientation()[1];
    imu_orientation_.y() = imu_sensor_.getOrientation()[2];
    imu_orientation_.z() = imu_sensor_.getOrientation()[3];

    // Filter the imu gyroscope and accelerometer
    imu_gyroscope_filt_ = imu_gyroscope_filter_.process(imu_gyroscope_);
    imu_accelerometer_filt_ = imu_accelerometer_filter_.process(imu_accelerometer_);
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
    if(!ground_truth_.getName().empty() && state_estimator_->getPositionEstimationType() == "ground_truth")
    {
        state_estimator_->setGroundTruthBasePosition(Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearPosition()));
        state_estimator_->setGroundTruthBaseLinearVelocity(Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearVelocity()));
    }
    if(!ground_truth_.getName().empty() && state_estimator_->getOrientationEstimationType() == "ground_truth")
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

    if(use_contact_sensors_)
        for(const auto& tmp : contact_sensors_)
        {
            tmp_vector3d_[0] = tmp.second.getForce()[0];
            tmp_vector3d_[1] = tmp.second.getForce()[1];
            tmp_vector3d_[2] = tmp.second.getForce()[2];
            state_estimator_->setContactForce(tmp.first,tmp_vector3d_);
            //state_estimator_->setContactState(tmp.first,tmp.second.getContactState());
        }

    state_estimator_->update(dt);

    // HACK
    estimator_->setJointVelocity(joint_velocities_filt_);
    estimator_->setJointPosition(joint_positions_);
    estimator_->setImuOrientation(imu_orientation_);
    estimator_->setImuAngularVelocities(imu_gyroscope_filt_);
    estimator_->setImuLinearAcceleration(imu_accelerometer_filt_);
    estimator_->setContactStates(state_estimator_->getContacts());
    estimator_->setContactForces(state_estimator_->getContactForces());
    estimator_->update(dt);
}

void Controller::updateStateMachine(const double &dt)
{
    desired_height_ = 0.0;
    current_height_ = robot_model_->getCurrentHeight();
    current_rpy_ = robot_model_->getBaseRotationInWorldRPY();
    unsigned int current_state = robot_model_->getState();
    double ramp;
    switch(current_state)
    {
      case(QuadrupedRobot::IDLE):
        if(posture_ == Controller::posture_t::UP)
        {
          init();
          robot_model_->setState(QuadrupedRobot::INIT);
          break;
        }
        break;

      case(QuadrupedRobot::INIT):
        des_joint_positions_ = robot_model_->getStandDownJointPostion();
        des_joint_velocities_.fill(0.0);
        ramp = ramp_init_->update(dt);
        // Linear interpolation between initial joint positions and desired (stand down)
        des_joint_positions_ = ramp * robot_model_->getStandDownJointPostion() + (1.0-ramp) * joint_positions_init_;
        updateImpedance(des_joint_positions_,des_joint_velocities_);
        if(ramp >= 1.0)
        {
          desired_yaw_ = robot_model_->getBaseRotationInWorldRPY().z();
          id_prob_->reset();
          ramp_init_->reset();
          robot_model_->setState(QuadrupedRobot::STANDING_UP);
        }
        break;

      case(QuadrupedRobot::STANDING_UP):
        updateComponents(dt);
        ramp = ramp_stand_up_->update(dt);
        desired_height_ = terrain_estimator_->getTerrainPositionWorld().z() + ramp * robot_model_->getStandUpHeight();
        des_joint_positions_ = ramp * robot_model_->getStandUpJointPostion() + (1.0-ramp) * robot_model_->getStandDownJointPostion();
        tmp_vector3d_ << 0.0, 0.0, desired_yaw_;
        rpyToRot(tmp_vector3d_,tmp_matrix3d_);
        tmp_vector3d_.setZero(); // com position
        tmp_vector3d_ << com_planner_->getComPosition().x(), com_planner_->getComPosition().y(), desired_height_;
        tmp_vector3d_1_.setZero(); // com velocity
        tmp_vector3d_1_.z() = foot_holds_planner_->getBaseLinearVelocityCmdZ();
        updateBaseReferences(tmp_vector3d_,tmp_vector3d_1_,tmp_matrix3d_);
        if(!updateSolver(des_joint_positions_))
        {
          robot_model_->setState(QuadrupedRobot::ANOMALY);
          break;
        }
        if(current_height_ >= terrain_estimator_->getTerrainPositionWorld().z() + robot_model_->getStandUpHeight())
        {
          foot_holds_planner_->reset();
          ramp_stand_up_->reset();
          com_planner_->resetVelocities();
          if(mode_ == Controller::mode_t::WALKING || mode_ == Controller::mode_t::MANIPULATION)
          {
            robot_model_->setState(QuadrupedRobot::ACTIVE);
            state_estimator_->startContactComputation();
            // HACK
            estimator_->init(joint_positions_,joint_velocities_filt_);
            break;
          }
        }
        break;

      case(QuadrupedRobot::ACTIVE):

        switch(mode_)
        {
        case Controller::mode_t::MANIPULATION:
          updateComponents(dt);
          updateBaseReferences(com_planner_->getComPosition(),com_planner_->getComVelocity(),foot_holds_planner_->getBaseRotationReference());
          id_prob_->setControlMode(IDProblem::mode_t::MANIPULATION);
          previous_mode_ = Controller::mode_t::MANIPULATION;
          break;
        case Controller::mode_t::WALKING:
          updateComponents(dt);
          updateBaseReferences(com_planner_->getComPosition(),com_planner_->getComVelocity(),foot_holds_planner_->getBaseRotationReference());
          id_prob_->setControlMode(IDProblem::mode_t::WALKING);
          previous_mode_ = Controller::mode_t::WALKING;
          break;
        case Controller::mode_t::RESET:
          foot_holds_planner_->setCmd(FootholdsPlanner::RESET_BASE);
          updateComponents(dt);
          tmp_vector3d_1_ << com_planner_->getComPosition().x(), com_planner_->getComPosition().y(), foot_holds_planner_->getBaseHeight(); // base position
          tmp_matrix3d_ = foot_holds_planner_->getBaseRotationReference(); // base orientation
          rotToRpy(tmp_matrix3d_,tmp_vector3d_);
          tmp_vector3d_2_.setZero(); // com velocity
          updateBaseReferences(tmp_vector3d_1_,tmp_vector3d_2_,tmp_matrix3d_);
          if( (current_height_ - previous_height_)/dt        <= EPS  &&
              std::abs(current_rpy_.x() - tmp_vector3d_.x()) <= 0.01 &&
              std::abs(current_rpy_.y() - tmp_vector3d_.y()) <= 0.01
              )
          {
            mode_ = previous_mode_;
            break;
          }
          break;
        };

        if(!updateSolver(robot_model_->getStandUpJointPostion()) || !performSafetyChecks())
        {
          robot_model_->setState(QuadrupedRobot::ANOMALY);
          break;
        }
        if(posture_ == Controller::posture_t::DOWN)
        {
          stand_down_starting_height_ = current_height_;
          robot_model_->setState(QuadrupedRobot::STANDING_DOWN);
          break;
        }
        break;

      case(QuadrupedRobot::STANDING_DOWN):
        updateComponents(dt);
        ramp = ramp_stand_down_->update(dt);
        desired_height_ = ramp * stand_down_starting_height_;
        des_joint_positions_ = (1.0-ramp) * robot_model_->getStandUpJointPostion() + (ramp) * robot_model_->getStandDownJointPostion();
        tmp_vector3d_ << 0.0, 0.0, robot_model_->getBaseRotationInWorldRPY().z();
        rpyToRot(tmp_vector3d_,tmp_matrix3d_);
        tmp_vector3d_.setZero(); // com position
        tmp_vector3d_ << com_planner_->getComPosition().x(), com_planner_->getComPosition().y(), desired_height_;
        tmp_vector3d_1_.setZero(); // com velocity
        tmp_vector3d_1_.z() = -foot_holds_planner_->getBaseLinearVelocityCmdZ();
        updateBaseReferences(tmp_vector3d_,tmp_vector3d_1_,tmp_matrix3d_);
        if(!updateSolver(des_joint_positions_))
        {
          ramp_stand_down_->reset();
          robot_model_->setState(QuadrupedRobot::ANOMALY);
          break;
        }
        if(desired_height_ <= EPS)
        {
          ramp_stand_down_->reset();
          //posture_ = Controller::posture_t::DOWN;
          robot_model_->setState(QuadrupedRobot::IDLE);
          state_estimator_->stopContactComputation();
          break;
        }
        break;

      case(QuadrupedRobot::ANOMALY):
        updateComponents(dt);
        des_joint_positions_ = robot_model_->getStandDownJointPostion();
        des_joint_velocities_.fill(0.0);
        updateImpedance(des_joint_positions_,des_joint_velocities_);
        if((current_height_ - previous_height_)/dt <= EPS)
        {
          posture_ = Controller::posture_t::DOWN;
          terrain_estimator_->reset();
          robot_model_->setState(QuadrupedRobot::IDLE);
          state_estimator_->stopContactComputation();
          break;
        }
        break;
    };

    previous_height_ = current_height_;
}
void Controller::init()
{
    // State estimator
    // Be sure to start the solver and the contact estimation when the robot is grounded.
    state_estimator_->resetGyroscopeIntegration();
    //state_estimator_->startContactComputation();
    state_estimator_->startHapticContactLoop();
    // Terrain Estimator
    //terrain_estimator_->reset();
    // Footholds planner with gait generator
    foot_holds_planner_->reset();
    foot_holds_planner_->setBasePosition(state_estimator_->getFloatingBasePosition());
    foot_holds_planner_->setDefaultBasePosition(Eigen::Vector3d(0.0,0.0,robot_model_->getStandUpHeight()));
    foot_holds_planner_->setBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
    foot_holds_planner_->setDefaultBaseOrientation(Eigen::Vector3d(0.0,0.0,0.0));
    foot_holds_planner_->initializeFeetPosition();
    // Filters
    imu_gyroscope_filter_.setTimeStep(period_);
    imu_accelerometer_filter_.setTimeStep(period_);
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
    // Fill initial joint positions
    joint_positions_init_ = joint_positions_;
}

void Controller::updateComponents(const double &dt)
{
  // Update the footholds planner
  foot_holds_planner_->update(dt);
  // Update the terrain estimator
  terrain_estimator_->computeTerrainEstimation(dt);
  // Update the CoM position and velocity reference
  com_planner_->update();
}

void Controller::updateBaseReferences(const Eigen::Vector3d &com_pos_ref, const Eigen::Vector3d &com_vel_ref, const Eigen::Matrix3d &orientation_ref)
{
  // Set the pose reference for the waist
  id_prob_->setWaistReference(orientation_ref,com_pos_ref.z(),com_vel_ref.z());
  // Set the velocity and position reference for the CoM in the solver
  id_prob_->setComReference(com_pos_ref,com_vel_ref);
}

bool Controller::performSafetyChecks()
{

  bool ok = true;

  // Check if we have at least one contact with the feet
  auto contacts = state_estimator_->getContacts();
  bool contact = false;
  auto foot_names = robot_model_->getFootNames();
  for(unsigned int i=0;i<foot_names.size();i++)
    contact = contact || contacts[foot_names[i]];
  if(!contact) // && state_estimator_->getFloatingBasePosition().z() > 0.3 * robot_model_->getStandUpHeight()
    contact_failures_cnt_->increase();
  else
    contact_failures_cnt_->reset();
  if(contact_failures_cnt_->upperLimitReached())
  {
    ok = false;
    ROS_WARN_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Lost contacts!");
  }

  // Check if the current joint velocities (only legs for the moment) are valid otherwise set robot state to anomaly
  std::vector<bool>&& checks = robot_model_->checkJointVelocities(joint_velocities_);
  auto leg_names = robot_model_->getLegNames();
  for(unsigned int j=0;j<leg_names.size();j++)
  {
    auto joints_idx = robot_model_->getLimbJointsIds(leg_names[j]);
    for(unsigned int i=0;i<joints_idx.size();i++)
    {
      if(checks[joints_idx[i]])
        velocity_lims_failures_cnt_[joints_idx[i]]->increase();
      else
        velocity_lims_failures_cnt_[joints_idx[i]]->reset();
      if(velocity_lims_failures_cnt_[joints_idx[i]]->upperLimitReached())
      {
        ok = false;
        auto names = robot_model_->getEnabledJointNames();
        ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_SEC,CLASS_NAME,"Reached joint velocity limit "<<names[joints_idx[i]]);
      }
    }
  }

  return ok;
}

void Controller::updateImpedance(const Eigen::VectorXd& des_joint_positions, const Eigen::VectorXd& des_joint_velocities)
{
  impedance_->update();
  des_joint_efforts_impedance_ = impedance_->getKp() * (des_joint_positions - joint_positions_) + impedance_->getKd() * ( des_joint_velocities - joint_velocities_);
}

bool Controller::updateSolver(const Eigen::VectorXd& des_joint_positions)
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

  // Update the postural
  //impedance_->startInertiaCompensation(true);
  impedance_->update();
  id_prob_->setPosture(impedance_->getKp(),impedance_->getKd(),des_joint_positions);
  //impedance_->startInertiaCompensation(false);

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
    // Update input devices
    devices_.writeToOutput(period.toSec());

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
    ROS_DEBUG_NAMED(CLASS_NAME,"Start the odomPublisher");

    // For base_footprint definition check here:
    // https://www.ros.org/reps/rep-0120.html#base-footprint
    // Create the following transformations:
    // base_footprint --> base
    //               `--> world (position available only if using ground_truth)

    Eigen::Affine3d world_T_base;
    Eigen::Vector3d tmp_v;
    double estimated_z;
    Eigen::Matrix3d tmp_R;
    Eigen::Quaterniond tmp_q;
    nav_msgs::Odometry odom;
    Eigen::Vector6d twist;

    ros::Time t_prev;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped basefoot_T_world;
    geometry_msgs::TransformStamped basefoot_T_base;
    static ros::Publisher odom_pub;
    odom_pub = root_nh_.advertise<nav_msgs::Odometry>("/spot/wolf_estimation/odom",100);

    basefoot_T_base.header.frame_id = BASE_FOOTPRINT_FRAME;
    basefoot_T_base.child_frame_id  = robot_model_->getBaseLinkName();

    basefoot_T_world.header.frame_id = BASE_FOOTPRINT_FRAME;
    basefoot_T_world.child_frame_id  = WORLD_FRAME_NAME;

    ros::Rate publishing_rate(250);

    while(!stopping_)
    {
        ros::Time t = ros::Time::now();

        if(t != t_prev) // Avoid publishing duplicated transforms
        {

          // Get base wrt the internal world estimation
          world_T_base = state_estimator_->getFloatingBasePose();
          // Get the estimated z of the base
          estimated_z = state_estimator_->getEstimatedBaseHeight();

          // Create the tf transform between base_footprint -> world
          tmp_v =  world_T_base.translation();
          tmp_v(2) = tmp_v(2) - estimated_z;
          rpyToRotTranspose(0.0,0.0,robot_model_->getBaseYawInWorld(),tmp_R);
          tmp_v = - tmp_R * tmp_v;
          tmp_q = tmp_R;
          // Set coordinates
          basefoot_T_world.transform.translation.x = tmp_v(0);
          basefoot_T_world.transform.translation.y = tmp_v(1);
          basefoot_T_world.transform.translation.z = tmp_v(2);
          basefoot_T_world.transform.rotation.w    = tmp_q.w();
          basefoot_T_world.transform.rotation.x    = tmp_q.x();
          basefoot_T_world.transform.rotation.y    = tmp_q.y();
          basefoot_T_world.transform.rotation.z    = tmp_q.z();
          // Set transform header
          basefoot_T_world.header.seq++;
          basefoot_T_world.header.stamp = t;

          br.sendTransform(basefoot_T_world);

          // Create the tf transform between base_footprint -> base
          tmp_q = robot_model_->getBaseRotationInHf();
          // Set coordinates
          basefoot_T_base.transform.translation.x = 0.0;
          basefoot_T_base.transform.translation.y = 0.0;
          basefoot_T_base.transform.translation.z = estimated_z;
          basefoot_T_base.transform.rotation.w    = tmp_q.w();
          basefoot_T_base.transform.rotation.x    = tmp_q.x();
          basefoot_T_base.transform.rotation.y    = tmp_q.y();
          basefoot_T_base.transform.rotation.z    = tmp_q.z();
          // Set transform header
          basefoot_T_base.header.seq++;
          basefoot_T_base.header.stamp = t;

          br.sendTransform(basefoot_T_base);

          // HACK
          world_T_base = estimator_->getBasePose();
          twist = estimator_->getBaseTwist();
          odom.header.seq              ++;
          odom.header.stamp            = t;
          odom.header.frame_id         = "world"; // FIXME
          odom.child_frame_id          = "base_link"; // FIXME
          odom.pose.pose               = tf2::toMsg(world_T_base);
          odom.twist.twist             = tf2::toMsg(twist);
          //tf2::eigenToCovariance(pose_cov,odom_msg_out_.pose.covariance);
          //tf2::eigenToCovariance(twist_cov,odom_msg_out_.twist.covariance);
          odom_pub.publish(odom);

        }

        //std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );

        t_prev = t;

        publishing_rate.sleep();
    }
    ROS_DEBUG_NAMED(CLASS_NAME,"Stop the odomPublisher");
}

void Controller::stopping(const ros::Time& /*time*/)
{
    ROS_DEBUG_NAMED(CLASS_NAME,"Stopping the Controller");

    stopping_ = true;
    odom_publisher_thread_->join();

    ROS_DEBUG_NAMED(CLASS_NAME,"Stopping Controller Completed");
}

void Controller::setBaseLinearVelocityCmdX(const double &v)
{
    vel_x_ = v;
    foot_holds_planner_->setBaseLinearVelocityCmdX(vel_x_);
}

void Controller::setBaseLinearVelocityCmdY(const double &v)
{
    vel_y_ = v;
    foot_holds_planner_->setBaseLinearVelocityCmdY(vel_y_);
}

void Controller::setBaseLinearVelocityCmdZ(const double &v)
{
    vel_z_ = v;
    foot_holds_planner_->setBaseLinearVelocityCmdZ(vel_z_);
}

void Controller::setBaseAngularVelocityCmdRoll(const double &v)
{
    vel_roll_ = v;
    foot_holds_planner_->setBaseAngularVelocityCmdRoll(vel_roll_);
}

void Controller::setBaseAngularVelocityCmdPitch(const double &v)
{
    vel_pitch_ = v;
    foot_holds_planner_->setBaseAngularVelocityCmdPitch(vel_pitch_);
}

void Controller::setBaseAngularVelocityCmdYaw(const double &v)
{
    vel_yaw_ = v;
    foot_holds_planner_->setBaseAngularVelocityCmdYaw(vel_yaw_);
}

double Controller::getBaseLinearVelocityCmdX()
{
    return vel_x_;
}

double Controller::getBaseLinearVelocityCmdY()
{
    return vel_y_;
}

double Controller::getBaseLinearVelocityCmdZ()
{
    return vel_z_;
}

double Controller::getBaseAngularVelocityCmdRoll()
{
    return vel_roll_;
}

double Controller::getBaseAngularVelocityCmdPitch()
{
    return vel_pitch_;
}

double Controller::getBaseAngularVelocityCmdYaw()
{
    return vel_yaw_;
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

Impedance* Controller::getImpedance() const
{
    return impedance_.get();
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
