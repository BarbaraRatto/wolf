#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>
#include <dls_controller/locomotion.h>

class JoyHandler
{

public:

    JoyHandler(ros::NodeHandle& node, std::shared_ptr<dls_controller::CommandsInterface> cmds)
    {
        joy_base_velocity_x_scale_     = 0.0;
        joy_base_velocity_y_scale_     = 0.0;
        joy_base_yaw_scale_            = 0.0;
        joy_base_pitch_scale_          = 0.0;
        joy_start_button_              = false;

        joy_sub_ = node.subscribe("joy", 1, &JoyHandler::joyCallback, this);

        assert(cmds);
        cmds_ = cmds;
    }

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy_base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
        joy_base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);

        joy_base_yaw_scale_         = static_cast<double>(msg->axes[2]);
        joy_base_pitch_scale_       = static_cast<double>(msg->axes[3]);

        joy_start_button_           = static_cast<bool>(msg->buttons[4]); // L1 button

       // Set the joypad commands
       if(std::abs(joy_base_velocity_x_scale_)>0 || std::abs(joy_base_velocity_y_scale_)>0)
       {
            cmds_->setCmd(dls_controller::CommandsInterface::BASE_LINEAR_VELOCITY);
            cmds_->setBaseVelocityScaleX(joy_base_velocity_x_scale_);
            cmds_->setBaseVelocityScaleY(joy_base_velocity_y_scale_);
            cmds_->setBaseVelocityScaleZ(0.0);
       }
       else if(std::abs(joy_base_yaw_scale_)>0)
       {
           cmds_->setCmd(dls_controller::CommandsInterface::BASE_ANGULAR_VELOCITY);
           cmds_->setBaseVelocityScaleYaw(joy_base_yaw_scale_);
       }
       else
       {
            cmds_->setCmd(dls_controller::CommandsInterface::HOLD);
            cmds_->setBaseVelocityScaleX(0.0);
            cmds_->setBaseVelocityScaleY(0.0);
            cmds_->setBaseVelocityScaleZ(0.0);
       }

    }

    /** @brief Ros subscriber for joypad */
    ros::Subscriber     joy_sub_;
    std::shared_ptr<dls_controller::CommandsInterface> cmds_;
    double joy_base_velocity_x_scale_;
    double joy_base_velocity_y_scale_;
    double joy_base_yaw_scale_;
    double joy_base_pitch_scale_;
    bool   joy_start_button_;
};


#endif
