#ifndef CONTROLLER_ROS_WRAPPER_H
#define CONTROLLER_ROS_WRAPPER_H

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
#include <wb_controller/ros_wrappers/ros_wrapper.h>
#include <wb_controller/geometry.h>

class ControllerRosWrapper : public RosWrapperInterface
{

public:

    typedef std::shared_ptr<ControllerRosWrapper> Ptr;

    ControllerRosWrapper(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, wb_controller::Controller* controller_ptr)
    {
        controller_ = controller_ptr;

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
            controller_->toggleBaseHeightControl();
            break;
        case 2:
            controller_->toggleInertiaCompensation();
            break;
        case 3:
            controller_->setDutyFactor(config.duty_factor);
            break;
        case 4:
            controller_->setGaitType(config.gaits);
            break;
        case 5:
            controller_->setSwingFrequency(config.swing_frequency);
            break;
        case 6:
            controller_->getFootholdsPlanner()->setLinearVelocity(config.base_linear_vel);
            ROS_INFO_STREAM("Set base linear velocity to "<< config.base_linear_vel);
            break;
        case 7:
            controller_->getFootholdsPlanner()->setAngularVelocity(config.base_angular_vel);
            ROS_INFO_STREAM("Set base angular velocity to "<< config.base_angular_vel);
            break;
        case 8:
            controller_->getFootholdsPlanner()->setStepHeight(config.step_height);
            break;
        case 9:
            controller_->getStateEstimator()->setContactThreshold(config.contact_force_th);
            ROS_INFO_STREAM("Set contact force threshold to "<< config.contact_force_th);
            break;
        case 10:
            //pid_scale_ = config.pid_scale;
            //ROS_INFO_STREAM("Set pid scale to "<< config.pid_scale);
            break;
        case 11:
            //cutoff_hz_qdot_ = config.cutoff_hz_qdot;
            //qdot_filter_.setOmega(2.0*M_PI*cutoff_hz_qdot_);
            //ROS_INFO_STREAM("Set cutoff frequency for qdot filter at "<< config.cutoff_hz_qdot);
            break;
        case 12:
            //cutoff_hz_gyro_ = config.cutoff_hz_gyro;
            //imu_gyroscope_filter_.setOmega(2.0*M_PI*cutoff_hz_gyro_);
            //ROS_INFO_STREAM("Set cutoff frequency  for gyroscope filter at "<< config.cutoff_hz_gyro);
            break;
        case 13:
            controller_->getLegsKinematics()->setClikGain(config.clik_gain);
            ROS_INFO_STREAM("Set x err gain at "<< config.clik_gain);
            break;
        case 14:
            // FIXME: this is not thread safe!
            // Kp swing
            //Kp_swing_leg_(0,0) = config.kp_haa_swing;
            //Kp_swing_leg_(1,1) = config.kp_hfe_swing;
            //Kp_swing_leg_(2,2) = config.kp_kfe_swing;
            //// Kd swing
            //Kd_swing_leg_(0,0) = config.kd_haa_swing;
            //Kd_swing_leg_(1,1) = config.kd_hfe_swing;
            //Kd_swing_leg_(2,2) = config.kd_kfe_swing;
            //// Kp stance
            //Kp_stance_leg_(0,0) = config.kp_haa_stance;
            //Kp_stance_leg_(1,1) = config.kp_hfe_stance;
            //Kp_stance_leg_(2,2) = config.kp_kfe_stance;
            //// Kd stance
            //Kd_stance_leg_(0,0) = config.kd_haa_stance;
            //Kd_stance_leg_(1,1) = config.kd_hfe_stance;
            //Kd_stance_leg_(2,2) = config.kd_kfe_stance;
            //ROS_INFO_NAMED(CLASS_NAME,"Set Kp and Kd for the postural");
            break;
        default:
            break;
        }
    }

    void dynamicReconfigureUpdate()
    {

        // Update the config for dynamic reconfigure
        //default_config_.toggle_solver = solver_started_;
        //default_config_.toggle_inertia_compensation = inertia_compensation_active_;
        //default_config_.pid_scale = pid_scale_;
        //default_config_.cutoff_hz_qdot = cutoff_hz_qdot_;
        //default_config_.cutoff_hz_gyro = cutoff_hz_gyro_;

        default_config_.kp_haa_swing   = controller_->Kp_swing_leg_(0,0);
        default_config_.kp_hfe_swing   = controller_->Kp_swing_leg_(1,1);
        default_config_.kp_kfe_swing   = controller_->Kp_swing_leg_(2,2);

        default_config_.kd_haa_swing   = controller_->Kd_swing_leg_(0,0);
        default_config_.kd_hfe_swing   = controller_->Kd_swing_leg_(1,1);
        default_config_.kd_kfe_swing   = controller_->Kd_swing_leg_(2,2);

        default_config_.kp_haa_stance = controller_->Kp_stance_leg_(0,0);
        default_config_.kp_hfe_stance = controller_->Kp_stance_leg_(1,1);
        default_config_.kp_kfe_stance = controller_->Kp_stance_leg_(2,2);

        default_config_.kd_haa_stance = controller_->Kd_stance_leg_(0,0);
        default_config_.kd_hfe_stance = controller_->Kd_stance_leg_(1,1);
        default_config_.kd_kfe_stance = controller_->Kd_stance_leg_(2,2);

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
            default_config_.toggle_base_height_control = controller_->getLegsKinematics()->isBaseHeightControlActive();
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
              contact_forces_pub_->msg_.contact[i] = controller_->getStateEstimator()->getContacts()[i];
              contact_forces_pub_->msg_.contact_positions[i].x = controller_->getStateEstimator()->getFeetPositionInWorld()[i](0);
              contact_forces_pub_->msg_.contact_positions[i].y = controller_->getStateEstimator()->getFeetPositionInWorld()[i](1);
              contact_forces_pub_->msg_.contact_positions[i].z = controller_->getStateEstimator()->getFeetPositionInWorld()[i](2);

              contact_forces_pub_->msg_.contact_forces[i].force.x = controller_->getStateEstimator()->getContactForces()[i](0);
              contact_forces_pub_->msg_.contact_forces[i].force.y = controller_->getStateEstimator()->getContactForces()[i](1);
              contact_forces_pub_->msg_.contact_forces[i].force.z = controller_->getStateEstimator()->getContactForces()[i](2);

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

#endif // CONTROLLER_ROS_WRAPPER_H

