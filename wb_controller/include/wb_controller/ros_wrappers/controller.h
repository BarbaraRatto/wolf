#ifndef ROS_WRAPPERS_CONTROLLER_H
#define ROS_WRAPPERS_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
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

// WBC
#include <wb_controller/controller.h>
#include <wb_controller/ros_wrappers/interface.h>
#include <wb_controller/geometry.h>

class ControllerRosWrapper : public RosWrapperInterface
{

public:

    typedef std::shared_ptr<ControllerRosWrapper> Ptr;

    ControllerRosWrapper(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, wb_controller::Controller* controller_ptr)
    {
        controller_ = controller_ptr;

        // Defaults
        double default_duty_factor = 0.3;
        if (!controller_nh.getParam("default_duty_factor", default_duty_factor))
        {
            ROS_WARN("No default duty factor given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_duty_factor);
        }
        double default_swing_frequency = 3.0; // [Hz]
        if (!controller_nh.getParam("default_swing_frequency", default_swing_frequency))
        {
            ROS_WARN("No default swing frequency given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_swing_frequency);
        }
        double default_contact_threshold = 50.0; // [N]
        if (!controller_nh.getParam("default_contact_threshold", default_contact_threshold))
        {
            ROS_WARN("No default contact threshold given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_contact_threshold);
        }
        double default_step_height = 0.05; // [m]
        if (!controller_nh.getParam("default_step_height", default_step_height))
        {
            ROS_WARN("No default step height given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_step_height);
        }
        double max_step_height = 0.15; // [m]
        if (!controller_nh.getParam("max_step_height", max_step_height))
        {
            ROS_WARN("No max step height given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_step_height);
        }
        double max_step_length = 0.5; // [m]
        if (!controller_nh.getParam("max_step_length", max_step_length))
        {
            ROS_WARN("No max step length given in namespace %s, using a max value of %f.", controller_nh.getNamespace().c_str(),max_step_length);
        }
        double default_base_linear_velocity = 0.5; // [m/s]
        if (!controller_nh.getParam("default_base_linear_velocity", default_base_linear_velocity))
        {
            ROS_WARN("No default base linear velocity given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_linear_velocity);
        }
        double default_base_angular_velocity = 0.5; // [rad/s]
        if (!controller_nh.getParam("default_base_angular_velocity", default_base_angular_velocity))
        {
            ROS_WARN("No default base angular velocity given in namespace %s, using a default value of %f.", controller_nh.getNamespace().c_str(),default_base_angular_velocity);
        }
        std::string estimation_position_type;
        if (!controller_nh.getParam("estimation_position_type", estimation_position_type))
            ROS_WARN("No default estimation_position_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),controller_->getStateEstimator()->getPositionEstimationType().c_str());
        else
            controller_->getStateEstimator()->setPositionEstimationType(estimation_position_type);

        std::string estimation_orientation_type;
        if (!controller_nh.getParam("estimation_orientation_type", estimation_orientation_type))
            ROS_WARN("No default estimation_orientation_type given in namespace %s, using %s", controller_nh.getNamespace().c_str(),controller_->getStateEstimator()->getOrientationEstimationType().c_str());
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

        // Set the callback for the dynamic reconfigure server
        server_.reset(new dynamic_reconfigure::Server<wb_controller::controllerConfig>(controller_nh));
        server_->setCallback(boost::bind(&ControllerRosWrapper::dynamicReconfigureCallback, this, _1, _2));
    }

    void dynamicReconfigureCallback(wb_controller::controllerConfig &config, uint32_t level)
    {
        switch(level)
        {
        case 0:
            controller_->toggleSolver();
            break;
        case 1:
            controller_->toggleInertiaCompensation();
            break;
        case 2:
            controller_->setDutyFactor(config.duty_factor);
            break;
        case 3:
            controller_->setGaitType(config.gaits);
            break;
        case 4:
            controller_->setSwingFrequency(config.swing_frequency);
            ROS_INFO_STREAM("Set swing frequency to "<< config.swing_frequency);
            break;
        case 5:
            controller_->getFootholdsPlanner()->setLinearVelocityCmd(config.base_linear_vel);
            ROS_INFO_STREAM("Set base linear velocity to "<< config.base_linear_vel);
            break;
        case 6:
            controller_->getFootholdsPlanner()->setAngularVelocityCmd(config.base_angular_vel);
            ROS_INFO_STREAM("Set base angular velocity to "<< config.base_angular_vel);
            break;
        case 7:
            controller_->getFootholdsPlanner()->setStepHeight(config.step_height);
             ROS_INFO_STREAM("Set step height to "<< config.step_height);
            break;
        case 8:
            controller_->getStateEstimator()->setContactThreshold(config.contact_force_th);
            ROS_INFO_STREAM("Set contact force threshold to "<< config.contact_force_th);
            break;
        case 9:
            controller_->getLegsKinematics()->setClikGain(config.clik_gain);
            ROS_INFO_STREAM("Set x err gain at "<< config.clik_gain);
            break;
        case 10:
            controller_->selectStack(config.stacks);
            ROS_INFO_STREAM("Selected stack "<< config.stacks);
            break;
        default:
            break;
        }
    }

    void dynamicReconfigureUpdate()
    {

        default_config_.stacks = "WALKING";

        if(controller_->getGaitGenerator())
        {
            default_config_.gaits = controller_->getGaitGenerator()->getGaitTypeName();
            default_config_.swing_frequency = controller_->getGaitGenerator()->getAvgSwingFrequency();
            default_config_.duty_factor = controller_->getGaitGenerator()->getAvgDutyFactor();
        }
        if(controller_->getFootholdsPlanner())
        {
            default_config_.base_linear_vel = controller_->getFootholdsPlanner()->getLinearVelocityCmd();
            default_config_.base_angular_vel = controller_->getFootholdsPlanner()->getAngularVelocityCmd();
            default_config_.step_height = controller_->getFootholdsPlanner()->getStepHeight();
        }
        if(controller_->getStateEstimator())
            default_config_.contact_force_th = controller_->getStateEstimator()->getContactThreshold();

        if(controller_->getLegsKinematics())
        {
            default_config_.clik_gain = controller_->getLegsKinematics()->getClikGain();
        }

        if(server_)
            server_->updateConfig(default_config_);
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

    };

protected:

    std::shared_ptr<dynamic_reconfigure::Server<wb_controller::controllerConfig>> server_;
    wb_controller::controllerConfig default_config_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::ContactForces>> contact_forces_pub_;
    /** @brief Real time publisher - foot holds */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::FootHolds>> foot_holds_pub_;
    wb_controller::Controller* controller_;

private:

};

#endif // ROS_WRAPPERS_CONTROLLER_H

