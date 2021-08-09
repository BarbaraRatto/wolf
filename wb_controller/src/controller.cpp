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

using namespace XBot;
using namespace Cartesian;
using namespace rt_logger;
using namespace rt_gui;

namespace wb_controller {

std::vector<std::string> _dof_names = {}; // To be loaded from the robot model
std::vector<std::string> _cartesian_names = {"x","y","z","roll","pitch","yaw"}; // This is our standard cartesian dofs order
std::vector<std::string> _joints_prefix = {"haa","hfe","kfe"};
std::vector<std::string> _legs_prefix = {"lf","lh","rf","rh"};

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
    contact_sensors_["lf_foot"] = cs_hw->getHandle("lf_foot_contact_sensor");
    contact_sensors_["rf_foot"] = cs_hw->getHandle("rf_foot_contact_sensor");
    contact_sensors_["lh_foot"] = cs_hw->getHandle("lh_foot_contact_sensor");
    contact_sensors_["rh_foot"] = cs_hw->getHandle("rh_foot_contact_sensor");
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
      ROS_ERROR_NAMED(CLASS_NAME,"PID gains must be positive!");
      return false;
    }
    ROS_DEBUG_NAMED(CLASS_NAME,"P value for joint %i is: %f",i,joint_p_gain_[i]);
    ROS_DEBUG_NAMED(CLASS_NAME,"I value for joint %i is: %f",i,joint_i_gain_[i]);
    ROS_DEBUG_NAMED(CLASS_NAME,"D value for joint %i is: %f",i,joint_d_gain_[i]);

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
      ROS_ERROR_NAMED(CLASS_NAME,"Kp and Kd gains must be positive!");
      return false;
    }
  }

  double default_clik_gain = 0.0; // Default value
  if (!controller_nh.getParam("gains/clik_gain", default_clik_gain))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No clik_gain given in the namespace: %s, set 0 as default value ", controller_nh.getNamespace().c_str());
  }

  // Initialize the inertia related matrices // FIXME
  robot_model_->getXBotModel()->getInertiaMatrix(M_);
  Mi_.setZero(M_.rows(), M_.cols());
  Kp_postural_.setIdentity(M_.rows(), M_.cols());
  Kd_postural_.setIdentity(M_.rows(), M_.cols());

  //Kp_postural_ = 30.0 * Kp_postural_;

  // Resize the variables
  joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  joint_velocities_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  joint_velocities_filt_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  joint_accellerations_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  joint_efforts_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  des_joint_positions_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  des_joint_velocities_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  des_joint_efforts_solver_.resize(static_cast<Eigen::Index>(joint_states_.size()+FLOATING_BASE_DOFS));
  des_joint_efforts_pids_.resize(static_cast<Eigen::Index>(joint_states_.size()));
  des_joint_efforts_.resize(static_cast<Eigen::Index>(joint_states_.size()));
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
  imu_orientation_.normalize();
  pid_scale_ = 1.0;

  gait_generator_.reset(new GaitGenerator(robot_model_->getFootNames(),Gait::TROT,"ellipse"));
  foot_holds_planner_.reset(new FootholdsPlanner(gait_generator_,robot_model_));
  state_estimator_.reset(new StateEstimator(gait_generator_,robot_model_));

  kin_.reset(new LegsKinematics(gait_generator_,robot_model_));
  kin_->setClikGain(default_clik_gain);
  //kin_->activateBaseHeightControl();
  des_joint_positions_ = kin_->getJointHomePositions();

  terrain_estimator_.reset(new TerrainEstimator(state_estimator_,foot_holds_planner_));
  terrain_estimator_->setMaxRoll(M_PI);
  terrain_estimator_->setMinRoll(-M_PI);
  terrain_estimator_->setMaxPitch(M_PI);
  terrain_estimator_->setMinPitch(-M_PI);

  com_planner_.reset(new ComPlanner(state_estimator_,foot_holds_planner_,terrain_estimator_));

  device_handler_.reset(new JoyHandler(controller_nh,this));

  // initialize the filters
  cutoff_hz_gyro_ = 300.;
  cutoff_hz_qdot_ = 300.;

  if(!root_nh.getParam("/task_period",period_)) // Get the initial task period
  {
    ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No task period given in namespace /");
    return false;
  }

  if(!root_nh.getParam("/internal_wrench",use_contact_sensors_)) // Use the contact sensors
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact estimation");
  else
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact sensors");

  qdot_filter_.setOmega(2.0*M_PI*cutoff_hz_qdot_);
  qdot_filter_.setDamping(1.0);
  qdot_filter_.setTimeStep(period_);

  imu_gyroscope_filter_.setOmega(2.0*M_PI*cutoff_hz_gyro_);
  imu_gyroscope_filter_.setDamping(1.0);
  imu_gyroscope_filter_.setTimeStep(period_);

  // Spawn the odom publisher thread
  odom_publisher_thread_.reset(new std::thread(&Controller::odomPublisher,this)); // FIXME

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/imu_gyroscope",imu_gyroscope_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/imu_gyroscope_filt",imu_gyroscope_filt_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/joint_velocities_",joint_velocities_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/joint_velocities_filt",joint_velocities_filt_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_efforts_solver",des_joint_efforts_solver_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_efforts_pids",des_joint_efforts_pids_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_joint_efforts",des_joint_efforts_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/des_base_rpy",des_base_rpy_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/period",period_);

  ros_wrapper_.reset(new ControllerRosWrapper(root_nh,controller_nh,this));

  return true;
}

bool Controller::setSwingFrequency(const double& swing_frequency)
{
  const std::vector<std::string>& foot_names = robot_model_->getFootNames();
  if(gait_generator_)
    for(unsigned int i=0; i < foot_names.size(); i++)
      gait_generator_->setSwingFrequency(foot_names[i],swing_frequency);
  else
  {
    ROS_WARN_NAMED(CLASS_NAME,"gait_generator not initialized yet.");
    return false;
  }
  return true;
}

bool Controller::selectStack(const std::string& stack)
{
  if(id_prob_)
  {
    if(stack == "WALKING")
      id_prob_->selectStack(IDProblem::stacks_t::WALKING);
    else if(stack == "MANIPULATION")
      id_prob_->selectStack(IDProblem::stacks_t::MANIPULATION);
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Wrong stack!");
      return false;
    }
  }

  return true;
}

void Controller::switchStack()
{
  if(id_prob_)
    id_prob_->switchStack();
  else
    ROS_WARN_NAMED(CLASS_NAME,"Did you press start?");
}

bool Controller::setGaitType(const std::string& gait_type)
{
  try
  {
    if(gait_generator_)
      gait_generator_->setGaitTypeName(gait_type);
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
      ROS_INFO_NAMED(CLASS_NAME,"Base height control is ON");
    else
      ROS_INFO_NAMED(CLASS_NAME,"Base height control is OFF");
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
    ROS_INFO_NAMED(CLASS_NAME,"Reset the solver");
    id_prob_.reset(new IDProblem(nh_,robot_model_));
    solver_created_ = true;
  }

  // Perform the init procedure
  init_done_ = false;

  solver_started_=!solver_started_;

  if(solver_started_)
    ROS_INFO_NAMED(CLASS_NAME,"Solver integration is ON");
  else
    ROS_INFO_NAMED(CLASS_NAME,"Solver integration is OFF");

}

void Controller::toggleInertiaCompensation()
{
  inertia_compensation_active_=!inertia_compensation_active_;

  if(inertia_compensation_active_)
    ROS_INFO_NAMED(CLASS_NAME,"Inertia compensation is ON");
  else
    ROS_INFO_NAMED(CLASS_NAME,"Inertia compensation is OFF");
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

  const std::vector<std::string>& foot_names = robot_model_->getFootNames();

  if(use_contact_sensors_)
    for(unsigned int i = 0; i<foot_names.size(); i++)
    {
       tmp_vector3d_[0] = contact_sensors_[foot_names[i]].getForce()[0];
       tmp_vector3d_[1] = contact_sensors_[foot_names[i]].getForce()[1];
       tmp_vector3d_[2] = contact_sensors_[foot_names[i]].getForce()[2];
       state_estimator_->setContactForces(foot_names[i],tmp_vector3d_);
    }

  state_estimator_->update(period.toSec());

  terrain_estimator_->computeTerrainEstimation(period.toSec());

  if(solver_started_) // Use the ID solver to calculate the torques
  {
    if(!init_done_) // FIXME Prepare a proper start up and rest procedure
    {
      // We need to set these values here because the robot is starting in the air with the simulation.
      // Be sure to start the solver and the contact estimation when the robot is grounded.
      state_estimator_->resetGyroscopeIntegration();
      state_estimator_->startContactsEstimation();
      state_estimator_->startHapticContactLoop();
      foot_holds_planner_->setBasePosition(state_estimator_->getFloatingBasePosition());
      foot_holds_planner_->setDefaultBasePosition(state_estimator_->getFloatingBasePosition());
      foot_holds_planner_->setBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
      foot_holds_planner_->setDefaultBaseOrientation(state_estimator_->getFloatingBaseOrientationRPY());
      foot_holds_planner_->initializeFeetPosition();

      kin_->reset();

      des_joint_positions_ = kin_->getJointHomePositions();
      des_joint_velocities_.fill(0.0);

      imu_gyroscope_filter_.setTimeStep(period_);
      qdot_filter_.setTimeStep(period_);

      // FIXME Why is it here?
      ros_wrapper_->dynamicReconfigureUpdate();

      init_done_ = true;
    }

    foot_holds_planner_->update(period.toSec()); // FIXME This should be done only after pid_scale_ = 0

    rotTorpy(foot_holds_planner_->getBaseRotationReference().transpose(),des_base_rpy_);
    // Set the pose reference for the waist
    id_prob_->setWaistReference(foot_holds_planner_->getBaseRotationReference(),foot_holds_planner_->getBaseHeight());

    // Set the velocity and position reference for the Com
    com_planner_->update(period.toSec());
    id_prob_->setComReference(com_planner_->getComPosition(),com_planner_->getComVelocity());

    id_prob_->setFrictionConesR(terrain_estimator_->getTerrainOrientationWorld().transpose());

    if(inertia_compensation_active_)
    {
      Mi_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,M_.rows()-FLOATING_BASE_DOFS,M_.cols()-FLOATING_BASE_DOFS)
          = M_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,M_.rows()-FLOATING_BASE_DOFS,M_.cols()-FLOATING_BASE_DOFS).inverse();
    }

    for(unsigned int i = 0; i<foot_names.size(); i++)
    {
      // Update the reference for the feet tasks, this is only used for visualization pourposes
      id_prob_->feet_[foot_names[i]]->setReference(gait_generator_->getReference(foot_names[i]),gait_generator_->getReferenceDot(foot_names[i])); //FIXME

      // FIXME I should spline the wrench limits to load correctly the legs in stance and unload the swinging leg
      // Set the wrench limits to enstablish the contacts
      if(gait_generator_->isSwinging(foot_names[i]))
      {
        id_prob_->swingWithFoot(foot_names[i]);
        ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Swinging: "<< foot_names[i]);

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

        id_prob_->stanceWithFoot(foot_names[i]);
        ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Stance: "<< foot_names[i]);
      }
    }

    id_prob_->postural_->setGains(Kp_postural_,Kd_postural_); // FIXME

    // Set the desired base height
    kin_->setDesiredBaseHeight(foot_holds_planner_->getBaseHeight());

    // Set the feed forward stance term with the terrain adjustment
    kin_->setFeedForwardStanceDot(terrain_estimator_->getPostureAdjustmentDot());

    // Update the desired joint positions from the ik and set that to the postural
    // task
    kin_->update(period.toSec(),joint_positions_);

    des_joint_positions_ = kin_->getDesiredJointPositions();
    des_joint_velocities_ = kin_->getDesiredJointVelocities();

    id_prob_->postural_->setReference(des_joint_positions_,des_joint_velocities_);

    // Get the solver solution
    if(!id_prob_->solve(des_joint_efforts_solver_))
    {
      ROS_WARN_NAMED(CLASS_NAME,"IDProblem::solve() skipping one step.");
      pid_active_ = true;
    }
    else
      pid_active_ = false;
  }
  else // Use a position PID controller
  {
    pid_active_ = true;
  }

  if(pid_active_)
  {
    des_joint_positions_ = kin_->getJointHomePositions();
    des_joint_velocities_.fill(0.0);
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
  ros_wrapper_->publish(time);

  RtLogger::getLogger().publish(time);
  RtGuiClient::getIstance().sync();
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

LegsKinematics* Controller::getLegsKinematics() const
{
  return kin_.get();
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

const Eigen::VectorXd& Controller::getDesiredJointEfforts() const
{
  return des_joint_efforts_solver_;
}

} //namespace
