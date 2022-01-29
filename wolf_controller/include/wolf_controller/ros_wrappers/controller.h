#ifndef ROS_WRAPPERS_CONTROLLER_H
#define ROS_WRAPPERS_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_srvs/Trigger.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <wolf_controller/utils.h>

// Generated
#include <wolf_controller/ContactForces.h>
#include <wolf_controller/FootHolds.h>
#include <wolf_controller/TerrainEstimation.h>
#include <wolf_controller/FrictionCones.h>

// WoLF
#include <wolf_controller/controller.h>
#include <wolf_controller/ros_wrappers/interface.h>
#include <wolf_controller/geometry.h>
#include <wolf_controller/utils.h>


class ControllerRosWrapper : public RosWrapperInterface
{

public:

    const std::string CLASS_NAME = "ControllerRosWrapper";

    typedef std::shared_ptr<ControllerRosWrapper> Ptr;

    ControllerRosWrapper(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, wolf_controller::Controller* controller_ptr)
    {
        controller_ = controller_ptr;

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
        double max_base_height = 0.5; // [m]
        if (!controller_nh.getParam("max_base_height", max_base_height))
        {
            ROS_WARN_NAMED(CLASS_NAME,"No max_base_height given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_base_height);
        }
        double default_base_linear_velocity = 0.5; // [m/s]
        if (!controller_nh.getParam("default_base_linear_velocity", default_base_linear_velocity))
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default_base_linear_velocity given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_linear_velocity);
        }
        double default_base_angular_velocity = 0.5; // [rad/s]
        if (!controller_nh.getParam("default_base_angular_velocity", default_base_angular_velocity))
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default_base_angular_velocity given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_angular_velocity);
        } 
        double default_friction_cones_mu = 0.7;
        if (!controller_nh.getParam("default_friction_cones_mu", default_friction_cones_mu))
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default_friction_cones_mu given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_friction_cones_mu);
        }
        double default_cutoff_freq_gyroscope = 300.;
        if (!controller_nh.getParam("default_cutoff_freq_gyroscope", default_cutoff_freq_gyroscope))
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default_cutoff_freq_gyroscope given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_cutoff_freq_gyroscope);
        }
        double default_cutoff_freq_qdot = 300.;
        if (!controller_nh.getParam("default_cutoff_freq_qdot", default_cutoff_freq_qdot))
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default_cutoff_freq_qdot given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_cutoff_freq_qdot);
        }

        Eigen::Vector3d k, dynamic_th, static_th;
        if (!controller_nh.getParam("push_recovery/k/x", k(0)) || // gains
            !controller_nh.getParam("push_recovery/k/y", k(1)) || // gains
            !controller_nh.getParam("push_recovery/k/r", k(2))  ) // gains
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default push_recovery/k given, proceeding without...");
            k = Eigen::Vector3d::Zero();
        }
        if (!controller_nh.getParam("push_recovery/dynamic_th/x", dynamic_th(0)) || // [m/s]
            !controller_nh.getParam("push_recovery/dynamic_th/y", dynamic_th(1)) || // [m/s]
            !controller_nh.getParam("push_recovery/dynamic_th/r", dynamic_th(2))  ) // [rad/s]
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default push_recovery/dynamic_th given, proceeding without...");
            dynamic_th = Eigen::Vector3d::Ones() * 1000.0;// dummy
        }
        if (!controller_nh.getParam("push_recovery/static_th/x", static_th(0)) || // [m/s]
            !controller_nh.getParam("push_recovery/static_th/y", static_th(1)) || // [m/s]
            !controller_nh.getParam("push_recovery/static_th/r", static_th(2))  ) // [rad/s]
        {
            ROS_WARN_NAMED(CLASS_NAME,"No default push_recovery/static_th given, proceeding without...");
            static_th = Eigen::Vector3d::Ones() * 1000.0; // dummy
        }

        controller_->getFootholdsPlanner()->setPushRecoveryGains(k(0),k(1),k(2));
        controller_->getFootholdsPlanner()->setPushRecoveryThresholds(static_th,dynamic_th);

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

        controller_->getGaitGenerator()->setSwingFrequency(default_swing_frequency);
        controller_->getGaitGenerator()->setDutyFactor(default_duty_factor);

        controller_->getFootholdsPlanner()->setBaseLinearVelocityCmd(default_base_linear_velocity);
        controller_->getFootholdsPlanner()->setBaseAngularVelocityCmd(default_base_angular_velocity);
        controller_->getFootholdsPlanner()->setStepHeight(default_step_height);
        controller_->getFootholdsPlanner()->setMaxStepHeight(max_step_height);
        controller_->getFootholdsPlanner()->setMaxStepLength(max_step_length);
        controller_->getFootholdsPlanner()->setMaxBaseHeight(max_base_height);

        controller_->getIDProblem()->setFrictionConesMu(default_friction_cones_mu);

        controller_->setCutoffFreqQdot(default_cutoff_freq_qdot);
        controller_->setCutoffFreqGyro(default_cutoff_freq_gyroscope);

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

        unsigned int n_contacts = controller_->getRobotModel()->getContactNames().size();
        contact_forces_pub_.reset(new realtime_tools::RealtimePublisher<wolf_controller::ContactForces>(controller_nh, "contact_forces", 4));
        contact_forces_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
        contact_forces_pub_->msg_.name.resize(n_contacts);
        contact_forces_pub_->msg_.contact.resize(n_contacts);
        contact_forces_pub_->msg_.contact_positions.resize(n_contacts);
        contact_forces_pub_->msg_.contact_forces.resize(n_contacts);
        contact_forces_pub_->msg_.des_contact_forces.resize(n_contacts);

        unsigned int n_feet = controller_->getRobotModel()->getNumberLegs();
        foot_holds_pub_.reset(new realtime_tools::RealtimePublisher<wolf_controller::FootHolds>(controller_nh, "foot_holds", 4));
        foot_holds_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
        foot_holds_pub_->msg_.name.resize(n_feet);
        foot_holds_pub_->msg_.desired_foothold.resize(n_feet);
        foot_holds_pub_->msg_.virtual_foothold.resize(n_feet);

        terrain_estimation_pub_.reset(new realtime_tools::RealtimePublisher<wolf_controller::TerrainEstimation>(controller_nh, "terrain_estimation", 4));
        terrain_estimation_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;

        friction_cones_pub_.reset(new realtime_tools::RealtimePublisher<wolf_controller::FrictionCones>(controller_nh, "friction_cones", 4));
        friction_cones_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
        friction_cones_pub_->msg_.foot_positions.resize(n_feet);
        friction_cones_pub_->msg_.cone_axis.resize(n_feet);
        friction_cones_pub_->msg_.mus.resize(n_feet);

        server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(controller_nh));
        server_->registerVariable<bool>("stand_up",false,boost::bind(&wolf_controller::Controller::standUp,controller_,_1),"stand up");
        server_->registerVariable<bool>("activate_push_recovery",controller_->getFootholdsPlanner()->isPushRecoveryActive(),boost::bind(&wolf_controller::FootholdsPlanner::startPushRecovery,controller_->getFootholdsPlanner(),_1),"activate push recovery");

        server_->registerVariable<double>("set_duty_factor",default_duty_factor,boost::bind(&wolf_controller::Controller::setDutyFactor,controller_,_1),"set duty factor",0.0,1.0);
        server_->registerVariable<double>("set_swing_frequency",default_swing_frequency,boost::bind(&wolf_controller::Controller::setSwingFrequency,controller_,_1),"set swing frequency",0.0,6.0);
        server_->registerVariable<double>("set_linear_vel",default_base_linear_velocity,boost::bind(&wolf_controller::FootholdsPlanner::setBaseLinearVelocityCmd,controller_->getFootholdsPlanner(),_1),"set linear velocity",0.0,1.0);
        server_->registerVariable<double>("set_angular_vel",default_base_angular_velocity,boost::bind(&wolf_controller::FootholdsPlanner::setBaseAngularVelocityCmd,controller_->getFootholdsPlanner(),_1),"set angular velocity",0.0,1.0);
        server_->registerVariable<double>("set_step_height",default_step_height,boost::bind(&wolf_controller::FootholdsPlanner::setStepHeight,controller_->getFootholdsPlanner(),_1),"set step height",0.0,max_step_height);
        server_->registerVariable<double>("set_contact_threshold",default_contact_threshold,boost::bind(&wolf_controller::StateEstimator::setContactThreshold,controller_->getStateEstimator(),_1),"set contact threshold",0.0,100.0);

        server_->registerEnumVariable<std::string>("select_gait","TROT",
                                                   boost::bind(&wolf_controller::Controller::selectGait,controller_,_1),
                                                   "select gait", {{"TROT","TROT"},{"CRAWL","CRAWL"}});

        server_->registerEnumVariable<std::string>("select_control_mode","WALKING",
                                                   boost::bind(&wolf_controller::Controller::selectControlMode,controller_,_1),
                                                   "select mode", {{"WALKING","WALKING"},{"MANIPULATION","MANIPULATION"}});

        server_->registerVariable<double>("set_mu",controller_->getIDProblem()->getFrictionConesMu(),boost::bind(&wolf_controller::Controller::setFrictionConesMu,controller_,_1),"set the friction cone value mu",0.0,1.0,controller_->getIDProblem()->CLASS_NAME);
        server_->registerVariable<double>("set_cutoff_freq_qdot",default_cutoff_freq_qdot,boost::bind(&wolf_controller::Controller::setCutoffFreqQdot,controller_,_1),"set cutoff frequency for the joint velocities",0,1000.0);
        server_->registerVariable<double>("set_cutoff_freq_gyroscope",default_cutoff_freq_gyroscope,boost::bind(&wolf_controller::Controller::setCutoffFreqGyro,controller_,_1),"set cutoff frequency for the imu gyroscope",0,1000.0);
        server_->publishServicesTopics();

        switch_control_mode_         = controller_nh.advertiseService("switch_control_mode",    &ControllerRosWrapper::switchControlModeCB,    this);
        switch_gait_                 = controller_nh.advertiseService("switch_gait",            &ControllerRosWrapper::switchGaitCB,           this);
        stand_up_srv_                = controller_nh.advertiseService("stand_up",               &ControllerRosWrapper::standUpCB,              this);
        stand_down_srv_              = controller_nh.advertiseService("stand_down",             &ControllerRosWrapper::standDownCB,            this);
        emergency_stop_srv_          = controller_nh.advertiseService("emergency_stop",         &ControllerRosWrapper::emergencyStopCB,        this);
        reset_base_srv_              = controller_nh.advertiseService("reset_base",             &ControllerRosWrapper::resetBaseCB,            this);
    }

    bool switchControlModeCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = true;
        controller_->switchControlMode();
        return res.success;
    }

    bool switchGaitCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = true;
        controller_->switchGait();
        return res.success;
    }

    bool emergencyStopCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = true;
        controller_->emergencyStop();
        return res.success;
    }

    bool resetBaseCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = true;
        controller_->resetBase();
        unsigned int current_state = controller_->getRobotModel()->getState();
        while(current_state == wolf_controller::QuadrupedRobot::RESET)
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

    bool standUpCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        res.success = true;
        controller_->selectPosture("UP");
        unsigned int current_state = controller_->getRobotModel()->getState();
        while(current_state != wolf_controller::QuadrupedRobot::WALKING      &&
              current_state != wolf_controller::QuadrupedRobot::MANIPULATION  )
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

    bool standDownCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
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

    virtual void publish(const ros::Time& time)
    {

      // FIXME it should not be there but for the moment I need it here because of the twist reset in the update of the solver:
      if(controller_->getIDProblem())
          controller_->getIDProblem()->publish(time);

      const std::vector<std::string>& contact_names = controller_->getRobotModel()->getContactNames();
      std::string current_contact_name;
      if(contact_forces_pub_.get() && contact_forces_pub_->trylock())
      {
          for(unsigned int i=0; i <contact_names.size(); i++)
          {
              current_contact_name = contact_names[i];
              contact_forces_pub_->msg_.name[i] = current_contact_name;
              contact_forces_pub_->msg_.contact[i] = controller_->getStateEstimator()->getContacts().at(current_contact_name);
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
    };

protected:

    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_controller::ContactForces>> contact_forces_pub_;
    /** @brief Real time publisher - foot holds */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_controller::FootHolds>> foot_holds_pub_;
    /** @brief Real time publisher - terrain estimation */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_controller::TerrainEstimation>> terrain_estimation_pub_;
    /** @brief Real time publisher - friction cones */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_controller::FrictionCones>> friction_cones_pub_;
    /** @brief Controller pnt */
    wolf_controller::Controller* controller_;
    /** @brief ROS services */
    ros::ServiceServer switch_control_mode_;
    ros::ServiceServer switch_gait_;
    ros::ServiceServer stand_up_srv_;
    ros::ServiceServer stand_down_srv_;
    ros::ServiceServer emergency_stop_srv_;
    ros::ServiceServer reset_base_srv_;

};

#endif // ROS_WRAPPERS_CONTROLLER_H

