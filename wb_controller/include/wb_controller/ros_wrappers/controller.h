#ifndef ROS_WRAPPERS_CONTROLLER_H
#define ROS_WRAPPERS_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <wb_controller/utils.h>

// ROS
#include <wb_controller/controllerConfig.h>
#include <wb_controller/ContactForces.h>
#include <wb_controller/FootHolds.h>
#include <wb_controller/TerrainEstimation.h>
#include <wb_controller/FrictionCones.h>

// WBC
#include <wb_controller/controller.h>
#include <wb_controller/ros_wrappers/interface.h>
#include <wb_controller/geometry.h>

class ControllerRosWrapper : public RosWrapperInterface
{

public:

    const std::string CLASS_NAME = "ControllerRosWrapper";

    typedef std::shared_ptr<ControllerRosWrapper> Ptr;

    ControllerRosWrapper(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, wb_controller::Controller* controller_ptr)
    {
        controller_ = controller_ptr;

        // Defaults
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
            ROS_WARN_NAMED(CLASS_NAME,"No default estimation_position_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),controller_->getStateEstimator()->getPositionEstimationType().c_str());
        else
            controller_->getStateEstimator()->setPositionEstimationType(estimation_position_type);

        std::string estimation_orientation_type;
        if (!controller_nh.getParam("estimation_orientation_type", estimation_orientation_type))
            ROS_WARN_NAMED(CLASS_NAME,"No default estimation_orientation_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),controller_->getStateEstimator()->getOrientationEstimationType().c_str());
        else
            controller_->getStateEstimator()->setOrientationEstimationType(estimation_orientation_type);

        controller_->getStateEstimator()->setContactThreshold(default_contact_threshold);

        controller_->getGaitGenerator()->setSwingFrequency(default_swing_frequency);
        controller_->getGaitGenerator()->setDutyFactor(default_duty_factor);

        controller_->getFootholdsPlanner()->setLinearVelocityCmd(default_base_linear_velocity);
        controller_->getFootholdsPlanner()->setAngularVelocityCmd(default_base_angular_velocity);
        controller_->getFootholdsPlanner()->setStepHeight(default_step_height);
        controller_->getFootholdsPlanner()->setMaxStepHeight(max_step_height);
        controller_->getFootholdsPlanner()->setMaxStepLength(max_step_length);

        unsigned int n_contacts = controller_->getRobotModel()->getContactNames().size();
        contact_forces_pub_.reset(new realtime_tools::RealtimePublisher<wb_controller::ContactForces>(controller_nh, "contact_forces", 4));
        contact_forces_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
        contact_forces_pub_->msg_.name.resize(n_contacts);
        contact_forces_pub_->msg_.contact.resize(n_contacts);
        contact_forces_pub_->msg_.contact_positions.resize(n_contacts);
        contact_forces_pub_->msg_.contact_forces.resize(n_contacts);
        contact_forces_pub_->msg_.des_contact_forces.resize(n_contacts);

        unsigned int n_feet = controller_->getRobotModel()->getNumberLegs();
        foot_holds_pub_.reset(new realtime_tools::RealtimePublisher<wb_controller::FootHolds>(controller_nh, "foot_holds", 4));
        foot_holds_pub_->msg_.header.frame_id = BASE_LINK_FRAME_NAME;
        foot_holds_pub_->msg_.name.resize(n_feet);
        foot_holds_pub_->msg_.desired_foothold.resize(n_feet);
        foot_holds_pub_->msg_.virtual_foothold.resize(n_feet);

        terrain_estimation_pub_.reset(new realtime_tools::RealtimePublisher<wb_controller::TerrainEstimation>(controller_nh, "terrain_estimation", 4));
        terrain_estimation_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;

        friction_cones_pub_.reset(new realtime_tools::RealtimePublisher<wb_controller::FrictionCones>(controller_nh, "friction_cones", 4));
        friction_cones_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
        friction_cones_pub_->msg_.foot_positions.resize(n_feet);
        friction_cones_pub_->msg_.cone_axis.resize(n_feet);
        friction_cones_pub_->msg_.mus.resize(n_feet);

        server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(controller_nh));
        server_->registerVariable<bool>("activate_solver",controller_->isSolverActive(),boost::bind(&wb_controller::Controller::startSolver,controller_,_1),"activate solver");
        server_->registerVariable<bool>("activate_inertia_compensation",controller_->isInertiaCompensationActive(),boost::bind(&wb_controller::Controller::startInertiaCompensation,controller_,_1),"activate inertia compensation");
        server_->registerVariable<bool>("activate_push_recovery",controller_->getFootholdsPlanner()->isPushRecoveryActive(),boost::bind(&wb_controller::FootholdsPlanner::startPushRecovery,controller_->getFootholdsPlanner(),_1),"activate push recovery");

        server_->registerVariable<double>("set_duty_factor",default_duty_factor,boost::bind(&wb_controller::Controller::setDutyFactor,controller_,_1),"set duty factor",0.0,1.0);
        server_->registerVariable<double>("set_swing_frequency",default_swing_frequency,boost::bind(&wb_controller::Controller::setSwingFrequency,controller_,_1),"set swing frequency",0.0,6.0);
        server_->registerVariable<double>("set_linear_vel",default_base_linear_velocity,boost::bind(&wb_controller::FootholdsPlanner::setLinearVelocityCmd,controller_->getFootholdsPlanner(),_1),"set linear velocity",0.0,1.0);
        server_->registerVariable<double>("set_angular_vel",default_base_angular_velocity,boost::bind(&wb_controller::FootholdsPlanner::setAngularVelocityCmd,controller_->getFootholdsPlanner(),_1),"set angular velocity",0.0,1.0);
        server_->registerVariable<double>("set_step_height",default_step_height,boost::bind(&wb_controller::FootholdsPlanner::setStepHeight,controller_->getFootholdsPlanner(),_1),"set step height",0.0,max_step_height);
        server_->registerVariable<double>("set_contact_threshold",default_contact_threshold,boost::bind(&wb_controller::StateEstimator::setContactThreshold,controller_->getStateEstimator(),_1),"set contact threshold",0.0,100.0);
        server_->registerVariable<double>("set_click_gain",controller_->getLegsKinematics()->getClikGain(),boost::bind(&wb_controller::LegsKinematics::setClikGain,controller_->getLegsKinematics(),_1),"set CLIK gain",0.0,1000.0);

        server_->registerEnumVariable<std::string>("select_gait","TROT",
                                                   boost::bind(&wb_controller::Controller::selectGait,controller_,_1),
                                                   "select gait", {{"TROT","TROT"},{"CRAWL","CRAWL"}});

        server_->registerEnumVariable<std::string>("select_stack","WALKING",
                                                   boost::bind(&wb_controller::Controller::selectStack,controller_,_1),
                                                   "select stack", {{"WALKING","WALKING"},{"MANIPULATION","MANIPULATION"}});
        server_->publishServicesTopics();
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
              contact_forces_pub_->msg_.contact_positions[i].x = controller_->getStateEstimator()->getContactPositionInWorld().at(current_contact_name)(0);
              contact_forces_pub_->msg_.contact_positions[i].y = controller_->getStateEstimator()->getContactPositionInWorld().at(current_contact_name)(1);
              contact_forces_pub_->msg_.contact_positions[i].z = controller_->getStateEstimator()->getContactPositionInWorld().at(current_contact_name)(2);

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
              friction_cones_pub_->msg_.foot_positions[i].x = controller_->getRobotModel()->getFeetPositionInWorld().at(foot_names[i]).x();
              friction_cones_pub_->msg_.foot_positions[i].y = controller_->getRobotModel()->getFeetPositionInWorld().at(foot_names[i]).y();
              friction_cones_pub_->msg_.foot_positions[i].z = controller_->getRobotModel()->getFeetPositionInWorld().at(foot_names[i]).z();

              friction_cones_pub_->msg_.cone_axis[i].x = controller_->getTerrainEstimator()->getTerrainNormal().x();
              friction_cones_pub_->msg_.cone_axis[i].y = controller_->getTerrainEstimator()->getTerrainNormal().y();
              friction_cones_pub_->msg_.cone_axis[i].z = controller_->getTerrainEstimator()->getTerrainNormal().z();
          }
          friction_cones_pub_->msg_.header.stamp = time;
          friction_cones_pub_->unlockAndPublish();
      }
    };

protected:

    wb_controller::controllerConfig default_config_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::ContactForces>> contact_forces_pub_;
    /** @brief Real time publisher - foot holds */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::FootHolds>> foot_holds_pub_;
    /** @brief Real time publisher - terrain estimation */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::TerrainEstimation>> terrain_estimation_pub_;
    /** @brief Real time publisher - friction cones */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::FrictionCones>> friction_cones_pub_;

    wb_controller::Controller* controller_;

private:

};

#endif // ROS_WRAPPERS_CONTROLLER_H

