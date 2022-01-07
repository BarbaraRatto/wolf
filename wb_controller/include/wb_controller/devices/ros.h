#ifndef DEVICES_ROS_H
#define DEVICES_ROS_H

#include <ros/ros.h>
#include <wb_controller/devices/interface.h>
#include <wb_controller/utils.h>
#include <wb_controller/controller.h>
#include <wb_controller/gait_generator.h>
#include <wb_controller/footholds_planner.h>
#include <wb_controller/id_problem.h>
#include <std_msgs/Bool.h>


template <typename msg_t>
class DeviceHandlerRosInterface : public DeviceHandlerInterface
{

public:

    DeviceHandlerRosInterface(ros::NodeHandle& node, wb_controller::Controller* controller_ptr, const std::string& topic)
        :DeviceHandlerInterface()
    {
        assert(controller_ptr);
        controller_ptr_ = controller_ptr;

        sub_ = node.subscribe(topic, 1, &DeviceHandlerRosInterface::callback, this);
        reset_sub_ = node.subscribe("reset_base", 1, &DeviceHandlerRosInterface::resetCallback, this);
    }

    virtual ~DeviceHandlerRosInterface() {}

    virtual void callback(const msg_t& msg) = 0;

protected:

    void update() // This work as a kind of state machine
    {
        // Clamp the scale values between -1 and 1
        if(base_velocity_x_scale_>1.0) base_velocity_x_scale_ = 1.0; if(base_velocity_x_scale_<-1.0) base_velocity_x_scale_ = -1.0;
        if(base_velocity_y_scale_>1.0) base_velocity_y_scale_ = 1.0; if(base_velocity_y_scale_<-1.0) base_velocity_y_scale_ = -1.0;
        if(base_velocity_z_scale_>1.0) base_velocity_z_scale_ = 1.0; if(base_velocity_z_scale_<-1.0) base_velocity_z_scale_ = -1.0;
        if(base_roll_scale_>1.0) base_roll_scale_ = 1.0; if(base_roll_scale_<-1.0) base_roll_scale_ = -1.0;
        if(base_pitch_scale_>1.0) base_pitch_scale_ = 1.0; if(base_pitch_scale_<-1.0) base_pitch_scale_ = -1.0;
        if(base_yaw_scale_>1.0) base_yaw_scale_ = 1.0; if(base_yaw_scale_<-1.0) base_yaw_scale_ = -1.0;

        unsigned int current_robot_state = controller_ptr_->getRobotModel()->getState();

        if(start_swing_ && current_robot_state == wb_controller::QuadrupedRobot::WALKING)
        {
            controller_ptr_->getFootholdsPlanner()->setCmd(wb_controller::FootholdsPlanner::LINEAR_AND_ANGULAR); // Start the swing
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleX(base_velocity_x_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleY(base_velocity_y_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleZ(base_velocity_z_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleYaw(base_yaw_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScalePitch(base_pitch_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleRoll(base_roll_scale_);
        }
        else if(reset_base_)
        {
            controller_ptr_->getFootholdsPlanner()->setCmd(wb_controller::FootholdsPlanner::RESET_BASE); // Reset the base orientation and position (height)
            reset_base_ = false;
        }
        else if(std::abs(base_velocity_z_scale_)>0 ||
                std::abs(base_yaw_scale_)       >0 ||
                std::abs(base_pitch_scale_)     >0 ||
                std::abs(base_roll_scale_)      >0  )
        {
            controller_ptr_->getFootholdsPlanner()->setCmd(wb_controller::FootholdsPlanner::BASE_ONLY); // Move the base orientation and Z
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleX(0.0);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleY(0.0);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleZ(base_velocity_z_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleYaw(base_yaw_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScalePitch(base_pitch_scale_);
            controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleRoll(base_roll_scale_);
        }
        else
        {
            controller_ptr_->getFootholdsPlanner()->setCmd(wb_controller::FootholdsPlanner::HOLD); // HODOR!
        }
    }

    void resetCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        base_velocity_x_scale_     = 0.;
        base_velocity_y_scale_     = 0.;
        base_velocity_z_scale_     = 0.;

        base_roll_scale_        = 0.;
        base_pitch_scale_       = 0.;
        base_yaw_scale_         = 0.;

        start_swing_ = false;

        reset_base_      = true;

        update();
    }

    wb_controller::Controller* controller_ptr_;
    /** @brief Ros subscriber for the device */
    ros::Subscriber sub_, reset_sub_;

};

#endif
