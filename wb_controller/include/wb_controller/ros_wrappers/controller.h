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

// WBC
#include <wb_controller/controller.h>

// WB
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

        controller_->getFootholdsPlanner()->setLinearVelocity(default_base_linear_velocity);
        controller_->getFootholdsPlanner()->setAngularVelocity(default_base_angular_velocity);
        controller_->getFootholdsPlanner()->setStepHeight(default_step_height);
        controller_->getFootholdsPlanner()->setMaxStepHeight(max_step_height);
        controller_->getFootholdsPlanner()->setMaxStepLength(max_step_length);

        // Create the realtime publishers
        //ci_joint_states_rt_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "ci/joint_states", 4));
        //ci_joint_states_rt_pub_->msg_.name.resize(wb_controller::_dof_names.size());
        //ci_joint_states_rt_pub_->msg_.position.resize(wb_controller::_dof_names.size());
        //ci_joint_states_rt_pub_->msg_.velocity.resize(wb_controller::_dof_names.size());
        //ci_joint_states_rt_pub_->msg_.effort.resize(wb_controller::_dof_names.size());
        //for (unsigned int i = 0; i < wb_controller::_dof_names.size(); i++)
        //    ci_joint_states_rt_pub_->msg_.name[i] = wb_controller::_dof_names[i];

        contact_forces_pub_.reset(new realtime_tools::RealtimePublisher<wb_controller::ContactForces>(controller_nh, "contact_forces", 4));
        contact_forces_pub_->msg_.header.frame_id = "world"; //FIXME
        contact_forces_pub_->msg_.name.resize(4);
        contact_forces_pub_->msg_.contact.resize(4);
        contact_forces_pub_->msg_.contact_positions.resize(4);
        contact_forces_pub_->msg_.contact_forces.resize(4);
        contact_forces_pub_->msg_.des_contact_forces.resize(4);

        joint_positions_xbot_.resize(static_cast<Eigen::Index>(wb_controller::_dof_names.size()));
        joint_velocities_xbot_.resize(static_cast<Eigen::Index>(wb_controller::_dof_names.size()));

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
            controller_->getFootholdsPlanner()->setLinearVelocity(config.base_linear_vel);
            ROS_INFO_STREAM("Set base linear velocity to "<< config.base_linear_vel);
            break;
        case 6:
            controller_->getFootholdsPlanner()->setAngularVelocity(config.base_angular_vel);
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
        default:
            break;
        }
    }

    void dynamicReconfigureUpdate()
    {

        if(controller_->getGaitGenerator())
        {
            default_config_.gaits = controller_->getGaitGenerator()->getGaitType();
            default_config_.swing_frequency = controller_->getGaitGenerator()->getSwingFrequency(controller_->getFeetNames()[0]); // FIXME - HACK
            default_config_.duty_factor = controller_->getGaitGenerator()->getDutyFactor(controller_->getFeetNames()[0]); // FIXME - HACK
        }
        if(controller_->getFootholdsPlanner())
        {
            default_config_.base_linear_vel = controller_->getFootholdsPlanner()->getLinearVelocity();
            default_config_.base_angular_vel = controller_->getFootholdsPlanner()->getAngularVelocity();
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

      const std::vector<std::string>& feet_names = controller_->getFeetNames();
      if(contact_forces_pub_.get() && contact_forces_pub_->trylock())
      {
          for(unsigned int i=0; i <feet_names.size(); i++)
          {
              contact_forces_pub_->msg_.name[i] = feet_names[i];
              contact_forces_pub_->msg_.contact[i] = controller_->getStateEstimator()->getContacts().at(feet_names[i]);
              contact_forces_pub_->msg_.contact_positions[i].x = controller_->getStateEstimator()->getFeetPositionInWorld().at(feet_names[i])(0);
              contact_forces_pub_->msg_.contact_positions[i].y = controller_->getStateEstimator()->getFeetPositionInWorld().at(feet_names[i])(1);
              contact_forces_pub_->msg_.contact_positions[i].z = controller_->getStateEstimator()->getFeetPositionInWorld().at(feet_names[i])(2);

              contact_forces_pub_->msg_.contact_forces[i].force.x = controller_->getStateEstimator()->getContactForces().at(feet_names[i])(0);
              contact_forces_pub_->msg_.contact_forces[i].force.y = controller_->getStateEstimator()->getContactForces().at(feet_names[i])(1);
              contact_forces_pub_->msg_.contact_forces[i].force.z = controller_->getStateEstimator()->getContactForces().at(feet_names[i])(2);

              // TODO
              //contact_forces_pub_->msg_.des_contact_forces[i].force.x = des_contact_forces_.segment(6*i,3)(0);
              //contact_forces_pub_->msg_.des_contact_forces[i].force.y = des_contact_forces_.segment(6*i,3)(1);
              //contact_forces_pub_->msg_.des_contact_forces[i].force.z = des_contact_forces_.segment(6*i,3)(2);
          }
          contact_forces_pub_->msg_.header.stamp = time;
          contact_forces_pub_->unlockAndPublish();
      }

      //if(ci_joint_states_rt_pub_.get() && ci_joint_states_rt_pub_->trylock())
      //{
      //    controller_->getXbotModel()->getJointPosition(joint_positions_xbot_);
      //    controller_->getXbotModel()->getJointVelocity(joint_velocities_xbot_);
      //
      //    for(unsigned int i = 0; i < joint_positions_.size(); i++)
      //    {
      //        ci_joint_states_rt_pub_->msg_.position[i]  = joint_positions_xbot_(i);
      //        ci_joint_states_rt_pub_->msg_.velocity[i]  = joint_velocities_xbot_(i);
      //        ci_joint_states_rt_pub_->msg_.effort[i]    = controller_->getDesiredJointEfforts()(i);
      //    }
      //    ci_joint_states_rt_pub_->msg_.header.stamp = time;
      //    ci_joint_states_rt_pub_->unlockAndPublish();
      //}




    };

protected:

    std::shared_ptr<dynamic_reconfigure::Server<wb_controller::controllerConfig>> server_;
    wb_controller::controllerConfig default_config_;
    /** @brief Real time publisher - desired joint states */
    //std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> ci_joint_states_rt_pub_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::ContactForces>> contact_forces_pub_;
    wb_controller::Controller* controller_;

private:

    /** @brief XBOT joint positions */
    Eigen::VectorXd joint_positions_xbot_;
    /** @brief XBOT joint velocities */
    Eigen::VectorXd joint_velocities_xbot_;
};

#endif // ROS_WRAPPERS_CONTROLLER_H

