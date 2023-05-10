/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the controller wrapper for ROS
 */

// WoLF
#include <wolf_controller/ros_wrappers/controller.h>

// RT GUI
#ifdef RT_GUI
#include <rt_gui/rt_gui_client.h>
using namespace rt_gui;
#endif

// System
#include <functional>


ControllerRosWrapper::ControllerRosWrapper(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, wolf_controller::Controller* const controller_ptr)
{
  controller_ = controller_ptr;

  // Set some ROS params
  controller_nh.setParam("robot_base_name",controller_ptr->getRobotModel()->getBaseLinkName());
  controller_nh.setParam("robot_foot_names",controller_ptr->getRobotModel()->getFootNames());
  controller_nh.setParam("robot_arm_names",controller_ptr->getRobotModel()->getArmNames());
  controller_nh.setParam("robot_imu_name",controller_ptr->getRobotModel()->getImuSensorName());

  // Defaults
  double default_duty_factor = 0.3;
  if (!controller_nh.getParam("default_duty_factor", default_duty_factor))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_duty_factor given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_duty_factor);
  }
  double default_swing_frequency = 3.0; // [Hz]
  if (!controller_nh.getParam("default_swing_frequency", default_swing_frequency))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_swing_frequency given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_swing_frequency);
  }
  double default_contact_threshold = 50.0; // [N]
  if (!controller_nh.getParam("default_contact_threshold", default_contact_threshold))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_contact_threshold given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_contact_threshold);
  }
  double default_step_height = 0.05; // [m]
  if (!controller_nh.getParam("default_step_height", default_step_height))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_step_height given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_step_height);
  }
  double max_step_height = 0.15; // [m]
  if (!controller_nh.getParam("max_step_height", max_step_height))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No max_step_height given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_step_height);
  }
  double max_step_length = 0.5; // [m]
  if (!controller_nh.getParam("max_step_length", max_step_length))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No max_step_length given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_step_length);
  }
  double default_step_reflex_contact_threshold = default_contact_threshold/3.0; // [N]
  if (!controller_nh.getParam("default_step_reflex_contact_threshold", default_step_reflex_contact_threshold))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_step_reflex_contact_threshold given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_step_reflex_contact_threshold);
  }
  double default_step_reflex_max_retraction = max_step_height/2.0; // [m]
  if (!controller_nh.getParam("default_step_reflex_max_retraction", default_step_reflex_max_retraction))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_step_reflex_max_retraction given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_step_reflex_max_retraction);
  }
  double max_base_height = 0.5; // [m]
  if (!controller_nh.getParam("max_base_height", max_base_height))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No max_base_height given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_base_height);
  }
  double max_base_roll = 2*M_PI; // [rad]
  if (!controller_nh.getParam("max_base_roll", max_base_roll))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No max_base_roll given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_base_roll);
  }
  double max_base_pitch = 2*M_PI; // [rad]
  if (!controller_nh.getParam("max_base_pitch", max_base_pitch))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No max_base_pitch given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_base_pitch);
  }
  double min_base_roll = -2*M_PI; // [rad]
  if (!controller_nh.getParam("min_base_roll", min_base_roll))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No min_base_roll given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),min_base_roll);
  }
  double min_base_pitch = -2*M_PI; // [rad]
  if (!controller_nh.getParam("min_base_pitch", min_base_pitch))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No min_base_pitch given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),min_base_pitch);
  }
  double default_base_linear_velocity, default_base_linear_velocity_x, default_base_linear_velocity_y, default_base_linear_velocity_z;
  default_base_linear_velocity = default_base_linear_velocity_x = default_base_linear_velocity_y = default_base_linear_velocity_z = 0.5; // [m/s]
  if (!controller_nh.getParam("default_base_linear_velocity", default_base_linear_velocity))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_base_linear_velocity given in namespace %s, looking for default_base_linear_velocity_[x,y,z].", controller_nh.getNamespace().c_str());
    if(!controller_nh.getParam("default_base_linear_velocity_x", default_base_linear_velocity_x) ||
       !controller_nh.getParam("default_base_linear_velocity_y", default_base_linear_velocity_y) ||
       !controller_nh.getParam("default_base_linear_velocity_z", default_base_linear_velocity_z)  )
      ROS_WARN_NAMED(CLASS_NAME,"No default_base_linear_velocity_[x,y,z] given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_linear_velocity);
  }
  else
    default_base_linear_velocity_x = default_base_linear_velocity_y = default_base_linear_velocity_z = default_base_linear_velocity;
  double default_base_angular_velocity, default_base_angular_velocity_roll, default_base_angular_velocity_pitch, default_base_angular_velocity_yaw;
  default_base_angular_velocity = default_base_angular_velocity_roll = default_base_angular_velocity_pitch = default_base_angular_velocity_yaw = 0.5; // [rad/s]
  if (!controller_nh.getParam("default_base_angular_velocity", default_base_angular_velocity))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_base_angular_velocity given in namespace %s, looking for default_base_angular_velocity_[roll,pitch,yaw].", controller_nh.getNamespace().c_str());
    if(!controller_nh.getParam("default_base_angular_velocity_roll", default_base_angular_velocity_roll)   ||
       !controller_nh.getParam("default_base_angular_velocity_pitch", default_base_angular_velocity_pitch) ||
       !controller_nh.getParam("default_base_angular_velocity_yaw", default_base_angular_velocity_yaw)     )
      ROS_WARN_NAMED(CLASS_NAME,"No default_base_angular_velocity_[roll,pitch,yaw] given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_angular_velocity);
  }
  else
    default_base_angular_velocity_roll = default_base_angular_velocity_pitch = default_base_angular_velocity_yaw = default_base_angular_velocity;

  double default_friction_cones_mu = 0.7;
  if (!controller_nh.getParam("default_friction_cones_mu", default_friction_cones_mu))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_friction_cones_mu given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_friction_cones_mu);
  }
  double default_cutoff_freq_gyroscope = 300.; // [Hz]
  if (!controller_nh.getParam("default_cutoff_freq_gyroscope", default_cutoff_freq_gyroscope))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_cutoff_freq_gyroscope given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_cutoff_freq_gyroscope);
  }
  double default_cutoff_freq_accelerometer = 300.; // [Hz]
  if (!controller_nh.getParam("default_cutoff_freq_accelerometer", default_cutoff_freq_accelerometer))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_cutoff_freq_accelerometer given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_cutoff_freq_accelerometer);
  }
  double default_cutoff_freq_qdot = 300.; // [Hz]
  if (!controller_nh.getParam("default_cutoff_freq_qdot", default_cutoff_freq_qdot))
  {
    ROS_WARN_NAMED(CLASS_NAME,"No default_cutoff_freq_qdot given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_cutoff_freq_qdot);
  }

  bool activate_com_z = true;
  controller_nh.getParam("activate_com_z", activate_com_z);
  controller_->getIDProblem()->activateComZ(activate_com_z);

  bool activate_postural = false;
  controller_nh.getParam("activate_postural", activate_postural);
  controller_->getIDProblem()->activatePostural(activate_postural);

  bool activate_angular_momentum = true;
  controller_nh.getParam("activate_angular_momentum", activate_angular_momentum);
  controller_->getIDProblem()->activateAngularMomentum(activate_angular_momentum);

  bool activate_joint_position_limits = false;
  controller_nh.getParam("activate_joint_position_limits", activate_joint_position_limits);
  controller_->getIDProblem()->activateJointPositionLimits(activate_joint_position_limits);

  double regularization = 1e-3;
  controller_nh.getParam("regularization", regularization);
  controller_->getIDProblem()->setRegularization(regularization);

  double min_forces_weight = 0.0;
  controller_nh.getParam("min_forces_weight", min_forces_weight);
  controller_->getIDProblem()->setForcesMinimizationWeight(min_forces_weight);

  double min_qddot_weight = 0.0;
  controller_nh.getParam("min_qddot_weight", min_qddot_weight);
  controller_->getIDProblem()->setJointAccelerationMinimizationWeight(min_qddot_weight);

  std::string estimation_position_type;
  if (!controller_nh.getParam("estimation_position_type", estimation_position_type))
    ROS_WARN_NAMED(CLASS_NAME,"No estimation_position_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),controller_->getStateEstimator()->getPositionEstimationType().c_str());
  else
    controller_->getStateEstimator()->setPositionEstimationType(estimation_position_type);

  std::string estimation_orientation_type;
  if (!controller_nh.getParam("estimation_orientation_type", estimation_orientation_type))
    ROS_WARN_NAMED(CLASS_NAME,"No estimation_orientation_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),controller_->getStateEstimator()->getOrientationEstimationType().c_str());
  else
    controller_->getStateEstimator()->setOrientationEstimationType(estimation_orientation_type);

  controller_->getStateEstimator()->setContactThreshold(default_contact_threshold);

  // Gait Generator
  controller_->getGaitGenerator()->setSwingFrequency(default_swing_frequency);
  controller_->getGaitGenerator()->setDutyFactor(default_duty_factor);
  controller_->getGaitGenerator()->setStepReflexContactThreshold(default_step_reflex_contact_threshold);
  controller_->getGaitGenerator()->setStepReflexMaxRetraction(default_step_reflex_max_retraction);

  // Footholds planner
  controller_->getFootholdsPlanner()->setBaseLinearVelocityCmd(default_base_linear_velocity_x,default_base_linear_velocity_y,default_base_linear_velocity_z);
  controller_->getFootholdsPlanner()->setBaseAngularVelocityCmd(default_base_angular_velocity_roll,default_base_angular_velocity_pitch,default_base_angular_velocity_yaw);
  controller_->getFootholdsPlanner()->setStepHeight(default_step_height);
  controller_->getFootholdsPlanner()->setMaxStepHeight(max_step_height);
  controller_->getFootholdsPlanner()->setMaxStepLength(max_step_length);
  controller_->getFootholdsPlanner()->setMaxBaseHeight(max_base_height);
  controller_->getFootholdsPlanner()->setMaxBaseRoll(max_base_roll);
  controller_->getFootholdsPlanner()->setMaxBasePitch(max_base_pitch);
  controller_->getFootholdsPlanner()->setMinBaseRoll(min_base_roll);
  controller_->getFootholdsPlanner()->setMinBasePitch(min_base_pitch);

  // ID problem
  controller_->getIDProblem()->setFrictionConesMu(default_friction_cones_mu);

  // Filters
  controller_->setCutoffFreqQdot(default_cutoff_freq_qdot);
  controller_->setCutoffFreqGyroscope(default_cutoff_freq_gyroscope);
  controller_->setCutoffFreqAccelerometer(default_cutoff_freq_accelerometer);

  // CMD interface
  controller_->setBaseLinearVelocityCmdX(default_base_linear_velocity_x);
  controller_->setBaseLinearVelocityCmdY(default_base_linear_velocity_y);
  controller_->setBaseLinearVelocityCmdZ(default_base_linear_velocity_z);
  controller_->setBaseAngularVelocityCmdRoll(default_base_angular_velocity_roll);
  controller_->setBaseAngularVelocityCmdPitch(default_base_angular_velocity_pitch);
  controller_->setBaseAngularVelocityCmdYaw(default_base_angular_velocity_yaw);

  bool activate_push_recovery = false;
  controller_nh.getParam("activate_push_recovery", activate_push_recovery);
  controller_->getFootholdsPlanner()->startPushRecovery(activate_push_recovery);

  bool activate_step_reflex = false;
  controller_nh.getParam("activate_step_reflex", activate_step_reflex);
  controller_->getGaitGenerator()->startStepReflex(activate_step_reflex);

  // Getting Kp and Kd gains
  // Legs
  Eigen::Vector3d Kp_leg, Kd_leg;
  Kp_leg = Kd_leg = Eigen::Vector3d::Ones();
  for(unsigned int i=0; i<wolf_controller::_joints_prefix.size(); i++)
  {
    if (!controller_nh.getParam("gains/Kp_leg/" + wolf_controller::_joints_prefix[i] , Kp_leg(i)))
    {
      ROS_WARN_NAMED(CLASS_NAME,"No default Kp_leg_%s gain given in the namespace: %s using 1.0 gain.",wolf_controller::_joints_prefix[i].c_str(),controller_nh.getNamespace().c_str());
    }
    if (!controller_nh.getParam("gains/Kd_leg/" + wolf_controller::_joints_prefix[i] , Kd_leg(i)))
    {
      ROS_WARN_NAMED(CLASS_NAME,"No default Kd_leg_%s gain given in the namespace: %s using 1.0 gain. ",wolf_controller::_joints_prefix[i].c_str(),controller_nh.getNamespace().c_str());
    }
    // Check if the values are positive
    if(Kp_leg(i)<0.0 || Kd_leg(i)<0.0)
    {
      ROS_WARN_NAMED(CLASS_NAME,"Kp_leg and Kd_leg gains must be positive!");
      Kp_leg(i) = Kd_leg(i) = 1.0;
    }
  }
  controller_ptr->getImpedance()->setLegsGains(Kp_leg,Kd_leg);
  // Arms
  if(controller_->getRobotModel()->getNumberArms() > 0)
  {
    unsigned int n_joint_arms = controller_->getRobotModel()->getLimbJointsIds(controller_->getRobotModel()->getArmNames()[0]).size();
    if(n_joint_arms>0)
    {
      Eigen::VectorXd Kp_arm, Kd_arm;
      Kp_arm.resize(n_joint_arms);
      Kd_arm.resize(n_joint_arms);
      Kp_arm.setOnes();
      Kd_arm.setOnes();
      for(unsigned int i=0; i<n_joint_arms; i++)
      {
        if (!controller_nh.getParam("gains/Kp_arm/j" + std::to_string(i) , Kp_arm(i)))
        {
          ROS_WARN_NAMED(CLASS_NAME,"No default Kp_arm_j%s gain given in the namespace: %s using 1.0 gain.",std::to_string(i).c_str(),controller_nh.getNamespace().c_str());
        }
        if (!controller_nh.getParam("gains/Kd_arm/j"  + std::to_string(i) , Kd_arm(i)))
        {
          ROS_WARN_NAMED(CLASS_NAME,"No default Kd_arm_j%s gain given in the namespace: %s using 1.0 gain. ",std::to_string(i).c_str(),controller_nh.getNamespace().c_str());
        }
        // Check if the values are positive
        if(Kp_arm(i)<0.0 || Kd_arm(i)<0.0)
        {
          ROS_WARN_NAMED(CLASS_NAME,"Kp_arm and Kd_arm gains must be positive!");
          Kp_arm(i) = Kd_arm(i) = 1.0;
        }
      }
      controller_ptr->getImpedance()->setArmsGains(Kp_arm,Kd_arm);
    }
  }

  // Real time publishers
  // Contact forces
  unsigned int n_contacts = controller_->getRobotModel()->getContactNames().size();
  contact_forces_pub_.reset(new realtime_tools::RealtimePublisher<wolf_msgs::ContactForces>(controller_nh, "contact_forces", 4));
  contact_forces_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  contact_forces_pub_->msg_.name.resize(n_contacts);
  contact_forces_pub_->msg_.contact.resize(n_contacts);
  contact_forces_pub_->msg_.des_contact.resize(n_contacts);
  contact_forces_pub_->msg_.contact_positions.resize(n_contacts);
  contact_forces_pub_->msg_.contact_forces.resize(n_contacts);
  contact_forces_pub_->msg_.des_contact_forces.resize(n_contacts);
  // Foot holds
  unsigned int n_feet = controller_->getRobotModel()->getNumberLegs();
  foot_holds_pub_.reset(new realtime_tools::RealtimePublisher<wolf_msgs::FootHolds>(controller_nh, "foot_holds", 4));
  foot_holds_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  foot_holds_pub_->msg_.name.resize(n_feet);
  foot_holds_pub_->msg_.desired_foothold.resize(n_feet);
  foot_holds_pub_->msg_.virtual_foothold.resize(n_feet);
  // Terrain estimation
  terrain_estimation_pub_.reset(new realtime_tools::RealtimePublisher<wolf_msgs::TerrainEstimation>(controller_nh, "terrain_estimation", 4));
  terrain_estimation_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
  // Friciton cones
  friction_cones_pub_.reset(new realtime_tools::RealtimePublisher<wolf_msgs::FrictionCones>(controller_nh, "friction_cones", 4));
  friction_cones_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  friction_cones_pub_->msg_.foot_positions.resize(n_feet);
  friction_cones_pub_->msg_.cone_axis.resize(n_feet);
  friction_cones_pub_->msg_.mus.resize(n_feet);
  // Capture point
  capture_point_pub_.reset(new realtime_tools::RealtimePublisher<wolf_msgs::CapturePoint>(controller_nh, "capture_point", 4));
  capture_point_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
  capture_point_pub_->msg_.support_polygon.points.resize(N_LEGS);
  // Controller state
  controller_state_pub_.reset(new realtime_tools::RealtimePublisher<wolf_msgs::ControllerState>(controller_nh, "controller_state", 4));
  controller_state_pub_->msg_.states = controller_->getRobotModel()->getStatesAsString();
  controller_state_pub_->msg_.current_state = controller_->getRobotModel()->getStateAsString();
  controller_state_pub_->msg_.modes = controller_->getModesAsString();
  controller_state_pub_->msg_.current_mode = controller_->getModeAsString();

  #ifdef OCS2
  // OCS2 mpc observation
  mpc_observation_pub_.reset(new realtime_tools::RealtimePublisher<ocs2_msgs::mpc_observation>(controller_nh, "mpc_observation", 4));
  mpc_observation_pub_->msg_.state.value.resize(24);
  mpc_observation_pub_->msg_.input.value.resize(48);
  mpc_observation_pub_->msg_.time = 0.0;
  mpc_observation_pub_->msg_.mode = 0;
  controller_->getRobotModel()->getJointPosition(tmp_vectorXd_);
  #endif

  // DDynamic reconfigure
  ddr_server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(controller_nh));
  ddr_server_->registerVariable<bool>("stand_up",false,boost::bind(&wolf_controller::Controller::standUp,controller_,_1),"stand up");
  ddr_server_->registerVariable<bool>("activate_push_recovery",controller_->getFootholdsPlanner()->isPushRecoveryActive(),boost::bind(&wolf_controller::FootholdsPlanner::startPushRecovery,controller_->getFootholdsPlanner(),_1),"activate push recovery");
  ddr_server_->registerVariable<bool>("activate_step_reflex",controller_->getGaitGenerator()->isStepReflexActive(),boost::bind(&wolf_controller::GaitGenerator::startStepReflex,controller_->getGaitGenerator(),_1),"activate step reflex");
  ddr_server_->registerVariable<double>("set_duty_factor",default_duty_factor,boost::bind(&wolf_controller::Controller::setDutyFactor,controller_,_1),"set duty factor",0.0,1.0);
  ddr_server_->registerVariable<double>("set_swing_frequency",default_swing_frequency,boost::bind(&wolf_controller::Controller::setSwingFrequency,controller_,_1),"set swing frequency",0.0,6.0);
  ddr_server_->registerVariable<double>("set_linear_vel_x",default_base_linear_velocity_x,boost::bind(&wolf_controller::Controller::setBaseLinearVelocityCmdX,controller_,_1),"set linear velocity x",0.0,1.0);
  ddr_server_->registerVariable<double>("set_linear_vel_y",default_base_linear_velocity_y,boost::bind(&wolf_controller::Controller::setBaseLinearVelocityCmdY,controller_,_1),"set linear velocity y",0.0,1.0);
  ddr_server_->registerVariable<double>("set_linear_vel_z",default_base_linear_velocity_z,boost::bind(&wolf_controller::Controller::setBaseLinearVelocityCmdZ,controller_,_1),"set linear velocity z",0.0,1.0);
  ddr_server_->registerVariable<double>("set_angular_vel_roll",default_base_angular_velocity_roll,boost::bind(&wolf_controller::Controller::setBaseAngularVelocityCmdRoll,controller_,_1),"set angular velocity roll",0.0,1.0);
  ddr_server_->registerVariable<double>("set_angular_vel_pitch",default_base_angular_velocity_pitch,boost::bind(&wolf_controller::Controller::setBaseAngularVelocityCmdPitch,controller_,_1),"set angular velocity pitch",0.0,1.0);
  ddr_server_->registerVariable<double>("set_angular_vel_yaw",default_base_angular_velocity_yaw,boost::bind(&wolf_controller::Controller::setBaseAngularVelocityCmdYaw,controller_,_1),"set angular velocity yaw",0.0,1.0);
  ddr_server_->registerVariable<double>("push_recovery_sensibility",controller_->getFootholdsPlanner()->getPushRecoverySensibility(),boost::bind(&wolf_controller::FootholdsPlanner::setPushRecoverySensibility,controller_->getFootholdsPlanner(),_1),"push recovery sensibility",0.0,1.0);
  //ddr_server_->registerVariable<double>("set_linear_vel",default_base_linear_velocity,boost::bind(&wolf_controller::FootholdsPlanner::setBaseLinearVelocityCmd,controller_->getFootholdsPlanner(),_1),"set linear velocity",0.0,1.0);
  //ddr_server_->registerVariable<double>("set_angular_vel",default_base_angular_velocity,boost::bind(&wolf_controller::FootholdsPlanner::setBaseAngularVelocityCmd,controller_->getFootholdsPlanner(),_1),"set angular velocity",0.0,1.0);
  ddr_server_->registerVariable<double>("set_step_height",default_step_height,boost::bind(&wolf_controller::FootholdsPlanner::setStepHeight,controller_->getFootholdsPlanner(),_1),"set step height",0.0,max_step_height);
  ddr_server_->registerVariable<double>("set_contact_threshold",default_contact_threshold,boost::bind(&wolf_controller::StateEstimator::setContactThreshold,controller_->getStateEstimator(),_1),"set contact threshold",0.0,100.0);

  ddr_server_->registerEnumVariable<std::string>("select_gait","TROT",
                                                 boost::bind(&wolf_controller::Controller::selectGait,controller_,_1),
                                                 "select gait", {{"TROT","TROT"},{"CRAWL","CRAWL"}});

  ddr_server_->registerEnumVariable<std::string>("select_control_mode","WPG",
                                                 boost::bind(&wolf_controller::Controller::selectControlMode,controller_,_1),
                                                 "select mode", {{"Walking pattern generator","WPG"},
                                                                 {"Model predictive control", "MPC"},
                                                                 {"External references",      "EXT"}
                                                 });

  ddr_server_->registerVariable<double>("set_mu",controller_->getIDProblem()->getFrictionConesMu(),boost::bind(&wolf_controller::Controller::setFrictionConesMu,controller_,_1),"set the friction cone value mu",0.0,1.0,controller_->getIDProblem()->CLASS_NAME);
  ddr_server_->registerVariable<double>("set_cutoff_freq_qdot",default_cutoff_freq_qdot,boost::bind(&wolf_controller::Controller::setCutoffFreqQdot,controller_,_1),"set cutoff frequency for the joint velocities",0,1000.0);
  ddr_server_->registerVariable<double>("set_cutoff_freq_gyroscope",default_cutoff_freq_gyroscope,boost::bind(&wolf_controller::Controller::setCutoffFreqGyroscope,controller_,_1),"set cutoff frequency for the imu gyroscope",0,1000.0);
  ddr_server_->registerVariable<double>("set_cutoff_freq_accelerometer",default_cutoff_freq_accelerometer,boost::bind(&wolf_controller::Controller::setCutoffFreqAccelerometer,controller_,_1),"set cutoff frequency for the imu accelerometer",0,1000.0);
  ddr_server_->publishServicesTopics();

  // ROS services
  switch_control_mode_         = controller_nh.advertiseService("switch_control_mode",         &ControllerRosWrapper::switchControlModeCB,         this);
  switch_gait_                 = controller_nh.advertiseService("switch_gait",                 &ControllerRosWrapper::switchGaitCB,                this);
  switch_posture_              = controller_nh.advertiseService("switch_posture",              &ControllerRosWrapper::switchPostureCB,             this);
  stand_up_srv_                = controller_nh.advertiseService("stand_up",                    &ControllerRosWrapper::standUpCB,                   this);
  stand_down_srv_              = controller_nh.advertiseService("stand_down",                  &ControllerRosWrapper::standDownCB,                 this);
  emergency_stop_srv_          = controller_nh.advertiseService("emergency_stop",              &ControllerRosWrapper::emergencyStopCB,             this);
  reset_base_srv_              = controller_nh.advertiseService("reset_base",                  &ControllerRosWrapper::resetBaseCB,                 this);
  decrease_step_height_        = controller_nh.advertiseService("decrease_step_height",        &ControllerRosWrapper::decreaseStepHeightCB,        this);
  increase_step_height_        = controller_nh.advertiseService("increase_step_height",        &ControllerRosWrapper::increaseStepHeightCB,        this);
  set_step_height_             = controller_nh.advertiseService("set_step_height",             &ControllerRosWrapper::setStepHeightCB,             this);
  activate_push_recovery_      = controller_nh.advertiseService("activate_push_recovery",      &ControllerRosWrapper::activatePushRecoveryCB,      this);
  activate_step_reflex_        = controller_nh.advertiseService("activate_step_reflex",        &ControllerRosWrapper::activateStepReflexCB,        this);
  increase_swing_frequency_    = controller_nh.advertiseService("increase_swing_frequency",    &ControllerRosWrapper::increaseSwingFrequencyCB,    this);
  decrease_swing_frequency_    = controller_nh.advertiseService("decrease_swing_frequency",    &ControllerRosWrapper::decreaseSwingFrequencyCB,    this);
  set_swing_frequency_         = controller_nh.advertiseService("set_swing_frequency",         &ControllerRosWrapper::setSwingFrequencyCB,         this);
  set_duty_factor_             = controller_nh.advertiseService("set_duty_factor",             &ControllerRosWrapper::setDutyFactorCB,             this);

  // RT GUI
#ifdef RT_GUI
  // create interface
  RtGuiClient::getIstance().init("wolf_panel","controller");
  RtGuiClient::getIstance().addTrigger(std::string("controller"),std::string("Stand up"),boost::bind(&wolf_controller::Controller::standUp,controller_,true));
  RtGuiClient::getIstance().addTrigger(std::string("controller"),std::string("Stand down"),boost::bind(&wolf_controller::Controller::standUp,controller_,false));
  RtGuiClient::getIstance().addTrigger(std::string("controller"),std::string("Emergency stop"),boost::bind(&wolf_controller::Controller::emergencyStop,controller_));
#endif
}

bool ControllerRosWrapper::increaseSwingFrequencyCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->getGaitGenerator()->increaseSwingFrequency();
  return res.success;
}

bool ControllerRosWrapper::decreaseSwingFrequencyCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->getGaitGenerator()->decreaseSwingFrequency();
  return res.success;
}

bool ControllerRosWrapper::setSwingFrequencyCB(wolf_msgs::float32Request& req, wolf_msgs::float32Response& res)
{
  res.success = true;
  if(req.data >= 0)
    controller_->setSwingFrequency(req.data);
  else
    res.success = false;
  return res.success;
}

bool ControllerRosWrapper::setDutyFactorCB(wolf_msgs::float32Request& req, wolf_msgs::float32Response& res)
{
  res.success = true;
  if(req.data >= 0)
    controller_->setDutyFactor(req.data);
  else
    res.success = false;
  return res.success;
}

bool ControllerRosWrapper::activatePushRecoveryCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->getFootholdsPlanner()->togglePushRecovery();
  return res.success;
}

bool ControllerRosWrapper::activateStepReflexCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->getGaitGenerator()->toggleStepReflex();
  return res.success;
}

bool ControllerRosWrapper::setStepHeightCB(wolf_msgs::float32Request& req, wolf_msgs::float32Response& res)
{
  res.success = true;
  if(req.data >= 0)
    controller_->getFootholdsPlanner()->setStepHeight(req.data);
  else
    res.success = false;
  return res.success;
}

bool ControllerRosWrapper::increaseStepHeightCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->getFootholdsPlanner()->increaseStepHeight();
  return res.success;
}

bool ControllerRosWrapper::decreaseStepHeightCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->getFootholdsPlanner()->decreaseStepHeight();
  return res.success;
}

bool ControllerRosWrapper::switchControlModeCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->switchControlMode();
  return res.success;
}

bool ControllerRosWrapper::switchGaitCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->switchGait();
  return res.success;
}

bool ControllerRosWrapper::switchPostureCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->switchPosture();
  return res.success;
}

bool ControllerRosWrapper::emergencyStopCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->emergencyStop();
  return res.success;
}

bool ControllerRosWrapper::resetBaseCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->resetBase();
  unsigned int current_mode = controller_->getControlMode();
  unsigned int current_state = controller_->getRobotModel()->getState();
  while(current_mode == wolf_controller::Controller::RESET)
  {
    if(current_state == wolf_controller::QuadrupedRobot::ANOMALY)
    {
      res.success = false;
      break;
    }
    current_mode = controller_->getControlMode();
    current_state = controller_->getRobotModel()->getState();
    std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );
  }
  return res.success;
}

bool ControllerRosWrapper::standUpCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->selectPosture("UP");
  unsigned int current_state = controller_->getRobotModel()->getState();
  while(current_state != wolf_controller::QuadrupedRobot::ACTIVE)
  {
    if(current_state == wolf_controller::QuadrupedRobot::ANOMALY)
    {
      res.success = false;
      break;
    }
    current_state = controller_->getRobotModel()->getState();
    std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );
  }
  return res.success;
}

bool ControllerRosWrapper::standDownCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  controller_->selectPosture("DOWN");
  unsigned int current_state = controller_->getRobotModel()->getState();
  while(current_state != wolf_controller::QuadrupedRobot::IDLE)
  {
    if(current_state == wolf_controller::QuadrupedRobot::ANOMALY)
    {
      res.success = false;
      break;
    }
    current_state = controller_->getRobotModel()->getState();
    std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );
  }
  return res.success;
}

void ControllerRosWrapper::publish(const ros::Time& time, const ros::Duration& period)
{
  if(controller_->getIDProblem())
    controller_->getIDProblem()->publish(time,period);

  if(controller_state_pub_.get() && controller_state_pub_->trylock())
  {
    controller_state_pub_->msg_.current_state = controller_->getRobotModel()->getStateAsString();
    controller_state_pub_->msg_.current_mode  = controller_->getModeAsString();
    controller_state_pub_->msg_.header.stamp  = time;
    controller_state_pub_->unlockAndPublish();
  }

  const std::vector<std::string>& foot_names = controller_->getRobotModel()->getFootNames();
  std::string current_foot_names;
  if(foot_holds_pub_.get() && foot_holds_pub_->trylock())
  {
    for(unsigned int i=0; i <foot_names.size(); i++)
    {
      current_foot_names = foot_names[i];
      foot_holds_pub_->msg_.name[i] = current_foot_names;
      foot_holds_pub_->msg_.desired_foothold[i].x = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).x();
      foot_holds_pub_->msg_.desired_foothold[i].y = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).y();
      foot_holds_pub_->msg_.desired_foothold[i].z = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).z();
      foot_holds_pub_->msg_.virtual_foothold[i].x = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).x();
      foot_holds_pub_->msg_.virtual_foothold[i].y = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).y();
      foot_holds_pub_->msg_.virtual_foothold[i].z = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).z();
    }
    foot_holds_pub_->msg_.header.stamp = time;
    foot_holds_pub_->unlockAndPublish();
  }

  // Note: des_contact is defined only for the feet at the moment (imagine des_contact as the planned contact state, which
  // for the feet is given by the state machine). For this reason we are using only the feet at the moment
  //const std::vector<std::string>& contact_names = controller_->getRobotModel()->getContactNames();
  //std::string current_contact_name;
  const std::vector<std::string>& contact_names = controller_->getRobotModel()->getContactNames();
  std::string current_contact_name;
  if(contact_forces_pub_.get() && contact_forces_pub_->trylock())
  {
    for(unsigned int i=0; i <contact_names.size(); i++)
    {
      current_contact_name = contact_names[i];
      contact_forces_pub_->msg_.name[i] = current_contact_name;
      contact_forces_pub_->msg_.contact[i] = controller_->getStateEstimator()->getContacts().at(current_contact_name);
      contact_forces_pub_->msg_.des_contact[i] = controller_->getDesiredContactStates()[i];
      contact_forces_pub_->msg_.contact_positions[i].x = controller_->getStateEstimator()->getContactPositionInBase().at(current_contact_name)(0);
      contact_forces_pub_->msg_.contact_positions[i].y = controller_->getStateEstimator()->getContactPositionInBase().at(current_contact_name)(1);
      contact_forces_pub_->msg_.contact_positions[i].z = controller_->getStateEstimator()->getContactPositionInBase().at(current_contact_name)(2);

      contact_forces_pub_->msg_.contact_forces[i].force.x = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(0);
      contact_forces_pub_->msg_.contact_forces[i].force.y = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(1);
      contact_forces_pub_->msg_.contact_forces[i].force.z = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(2);

      contact_forces_pub_->msg_.des_contact_forces[i].force.x = controller_->getDesiredContactForces()[i](0);
      contact_forces_pub_->msg_.des_contact_forces[i].force.y = controller_->getDesiredContactForces()[i](1);
      contact_forces_pub_->msg_.des_contact_forces[i].force.z = controller_->getDesiredContactForces()[i](2);
    }
    contact_forces_pub_->msg_.header.stamp = time;
    contact_forces_pub_->unlockAndPublish();
  }

  if(terrain_estimation_pub_.get() && terrain_estimation_pub_->trylock())
  {
    for(unsigned int i=0; i <foot_names.size(); i++)
    {
      terrain_estimation_pub_->msg_.central_point.x = controller_->getTerrainEstimator()->getTerrainPositionWorld().x();
      terrain_estimation_pub_->msg_.central_point.y = controller_->getTerrainEstimator()->getTerrainPositionWorld().y();
      terrain_estimation_pub_->msg_.central_point.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();

      terrain_estimation_pub_->msg_.terrain_normal.x = controller_->getTerrainEstimator()->getTerrainNormal().x();
      terrain_estimation_pub_->msg_.terrain_normal.y = controller_->getTerrainEstimator()->getTerrainNormal().y();
      terrain_estimation_pub_->msg_.terrain_normal.z = controller_->getTerrainEstimator()->getTerrainNormal().z();
    }
    terrain_estimation_pub_->msg_.header.stamp = time;
    terrain_estimation_pub_->unlockAndPublish();
  }

  if(friction_cones_pub_.get() && friction_cones_pub_->trylock())
  {
    for(unsigned int i=0; i <foot_names.size(); i++)
    {
      friction_cones_pub_->msg_.foot_positions[i].x = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).x();
      friction_cones_pub_->msg_.foot_positions[i].y = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).y();
      friction_cones_pub_->msg_.foot_positions[i].z = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).z();

      friction_cones_pub_->msg_.cone_axis[i].x = controller_->getTerrainEstimator()->getTerrainNormal().x();
      friction_cones_pub_->msg_.cone_axis[i].y = controller_->getTerrainEstimator()->getTerrainNormal().y();
      friction_cones_pub_->msg_.cone_axis[i].z = controller_->getTerrainEstimator()->getTerrainNormal().z();

      friction_cones_pub_->msg_.mus[i].data = (controller_->getIDProblem() ? controller_->getIDProblem()->getFrictionConesMu() : 1.0);
    }
    friction_cones_pub_->msg_.header.stamp = time;
    friction_cones_pub_->unlockAndPublish();
  }

  if(capture_point_pub_.get() && capture_point_pub_->trylock())
  {
    const std::vector<std::string>& ordered_foot_names = controller_->getFootholdsPlanner()->getPushRecovery()->getOrderedFootNames();

    for(unsigned int i=0; i <N_LEGS; i++)
    {
      capture_point_pub_->msg_.support_polygon.points[i].x = controller_->getFootholdsPlanner()->getPushRecovery()->getSupportPolygonEdges()[i].x();
      capture_point_pub_->msg_.support_polygon.points[i].y = controller_->getFootholdsPlanner()->getPushRecovery()->getSupportPolygonEdges()[i].y();
      capture_point_pub_->msg_.support_polygon.points[i].z = controller_->getRobotModel()->getFootPositionInWorld(ordered_foot_names[i]).z();
    }
    capture_point_pub_->msg_.com.x = controller_->getFootholdsPlanner()->getPushRecovery()->getComPositionXY().x();
    capture_point_pub_->msg_.com.y = controller_->getFootholdsPlanner()->getPushRecovery()->getComPositionXY().y();
    capture_point_pub_->msg_.com.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();
    capture_point_pub_->msg_.capture_point.x = controller_->getFootholdsPlanner()->getPushRecovery()->getCapturePoint().x();
    capture_point_pub_->msg_.capture_point.y = controller_->getFootholdsPlanner()->getPushRecovery()->getCapturePoint().y();
    capture_point_pub_->msg_.capture_point.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();
    capture_point_pub_->msg_.header.stamp = time;
    capture_point_pub_->unlockAndPublish();
  }

  #ifdef OCS2
  if(mpc_observation_pub_.get() && mpc_observation_pub_->trylock())
  {

    controller_->getRobotModel()->getCentroidalMomentum(tmp_vector6d_);
    controller_->getRobotModel()->getFloatingBasePose(tmp_affine3d_);
    double robot_mass = controller_->getRobotModel()->getMass();

    // STATE
    // linear momentum
    mpc_observation_pub_->msg_.state.value[0] = tmp_vector6d_(0) / robot_mass;
    mpc_observation_pub_->msg_.state.value[1] = tmp_vector6d_(1) / robot_mass;
    mpc_observation_pub_->msg_.state.value[2] = tmp_vector6d_(2) / robot_mass;

    // angular momentum
    mpc_observation_pub_->msg_.state.value[3] = tmp_vector6d_(3) / robot_mass;
    mpc_observation_pub_->msg_.state.value[4] = tmp_vector6d_(4) / robot_mass;
    mpc_observation_pub_->msg_.state.value[5] = tmp_vector6d_(5) / robot_mass;

    // base position
    mpc_observation_pub_->msg_.state.value[6] = tmp_affine3d_.translation().x();
    mpc_observation_pub_->msg_.state.value[7] = tmp_affine3d_.translation().y();
    mpc_observation_pub_->msg_.state.value[8] = tmp_affine3d_.translation().z();

    // base orientation (double check that)
    tmp_vector3d_ = wolf_controller_utils::rotToRpy(tmp_affine3d_.linear());
    mpc_observation_pub_->msg_.state.value[9]  = tmp_vector3d_.z();
    mpc_observation_pub_->msg_.state.value[10] = tmp_vector3d_.y();
    mpc_observation_pub_->msg_.state.value[11] = tmp_vector3d_.x();

    // joint positions (check the order, also be sure that we are not taking the arm joints)
    controller_->getRobotModel()->getJointPosition(tmp_vectorXd_);
    for(unsigned int i=0; i<12; i++)
      mpc_observation_pub_->msg_.state.value[12 + i] = tmp_vectorXd_(i+FLOATING_BASE_DOFS);

    // INPUT
    // contact forces (FIXME hardcoded names and order)
    mpc_observation_pub_->msg_.input.value[0] = controller_->getStateEstimator()->getContactForce("lf_foot").x();
    mpc_observation_pub_->msg_.input.value[1] = controller_->getStateEstimator()->getContactForce("lf_foot").y();
    mpc_observation_pub_->msg_.input.value[2] = controller_->getStateEstimator()->getContactForce("lf_foot").z();

    mpc_observation_pub_->msg_.input.value[3] = controller_->getStateEstimator()->getContactForce("lh_foot").x();
    mpc_observation_pub_->msg_.input.value[4] = controller_->getStateEstimator()->getContactForce("lh_foot").y();
    mpc_observation_pub_->msg_.input.value[5] = controller_->getStateEstimator()->getContactForce("lh_foot").z();

    mpc_observation_pub_->msg_.input.value[6] = controller_->getStateEstimator()->getContactForce("rf_foot").x();
    mpc_observation_pub_->msg_.input.value[7] = controller_->getStateEstimator()->getContactForce("rf_foot").y();
    mpc_observation_pub_->msg_.input.value[8] = controller_->getStateEstimator()->getContactForce("rf_foot").z();

    mpc_observation_pub_->msg_.input.value[9]  = controller_->getStateEstimator()->getContactForce("rh_foot").x();
    mpc_observation_pub_->msg_.input.value[10] = controller_->getStateEstimator()->getContactForce("rh_foot").y();
    mpc_observation_pub_->msg_.input.value[11] = controller_->getStateEstimator()->getContactForce("rh_foot").z();

    // joints velocities
    controller_->getRobotModel()->getJointVelocity(tmp_vectorXd_);
    for(unsigned int i=0; i<12; i++)
      mpc_observation_pub_->msg_.input.value[36 + i] = tmp_vectorXd_(i+FLOATING_BASE_DOFS);

    // time
    mpc_observation_pub_->msg_.time = mpc_observation_pub_->msg_.time + period.toSec();

    // mode (FIXME hardcoded names)
    mpc_observation_pub_->msg_.mode = static_cast<int8_t>(controller_->getStateEstimator()->getContact("lf_foot"))*8
                                    + static_cast<int8_t>(controller_->getStateEstimator()->getContact("lh_foot"))*4
                                    + static_cast<int8_t>(controller_->getStateEstimator()->getContact("rf_foot"))*2
                                    + static_cast<int8_t>(controller_->getStateEstimator()->getContact("rh_foot"));

    mpc_observation_pub_->unlockAndPublish();
  }
  #endif

}
