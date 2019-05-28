#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>
#include <dls_controller/locomotion.h>

class JoyHandler
{

public:

    /**
     * @brief Shared pointer to JoyHandler
     */
    typedef std::shared_ptr<JoyHandler> Ptr;

    /**
     * @brief Shared pointer to const JoyHandler
     */
    typedef std::shared_ptr<const JoyHandler> ConstPtr;

    JoyHandler(ros::NodeHandle& node, std::shared_ptr<dls_controller::CommandsInterface> cmds)
    {
        joy_base_velocity_x_scale_     = 0.0;
        joy_base_velocity_y_scale_     = 0.0;
        joy_base_yaw_scale_            = 0.0;
        joy_base_pitch_scale_          = 0.0;
        joy_start_button_              = false;
        joy_reset_button_              = false;

        joy_sub_ = node.subscribe("joy", 1, &JoyHandler::joyCallback, this);

        assert(cmds);
        cmds_ = cmds;
    }

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy_base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
        joy_base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
        joy_base_velocity_z_scale_     = (static_cast<double>(msg->buttons[5])-static_cast<double>(msg->buttons[7])); //R1 and R2

        joy_base_yaw_scale_         = static_cast<double>(msg->axes[2]);
        joy_base_pitch_scale_       = static_cast<double>(msg->axes[3]);
        joy_base_roll_scale_       = static_cast<double>(msg->axes[4]);

        joy_start_button_           = static_cast<bool>(msg->buttons[4]); // L1 button
        joy_reset_button_           = static_cast<bool>(msg->buttons[6]); // L2 button

       if(joy_start_button_)
       {
           cmds_->setCmd(dls_controller::CommandsInterface::LINEAR_AND_ANGULAR); // Start the swing
           cmds_->setBaseVelocityScaleX(joy_base_velocity_x_scale_);
           cmds_->setBaseVelocityScaleY(joy_base_velocity_y_scale_);
           cmds_->setBaseVelocityScaleZ(joy_base_velocity_z_scale_);
           cmds_->setBaseVelocityScaleYaw(joy_base_yaw_scale_);
           cmds_->setBaseVelocityScalePitch(joy_base_pitch_scale_);
           cmds_->setBaseVelocityScaleRoll(joy_base_roll_scale_);
       }
       else if(joy_reset_button_)
       {
           cmds_->setCmd(dls_controller::CommandsInterface::RESET_BASE); // Reset the base orientation and position (height)
       }
       else if(std::abs(joy_base_velocity_z_scale_)>0 ||
               std::abs(joy_base_yaw_scale_)       >0 ||
               std::abs(joy_base_pitch_scale_)     >0 ||
               std::abs(joy_base_roll_scale_)      >0  )
       {
           cmds_->setCmd(dls_controller::CommandsInterface::BASE_ONLY); // Move the base orientation
           cmds_->setBaseVelocityScaleX(0.0);
           cmds_->setBaseVelocityScaleY(0.0);
           cmds_->setBaseVelocityScaleZ(joy_base_velocity_z_scale_);
           cmds_->setBaseVelocityScaleYaw(joy_base_yaw_scale_);
           cmds_->setBaseVelocityScalePitch(joy_base_pitch_scale_);
           cmds_->setBaseVelocityScaleRoll(joy_base_roll_scale_);
       }
       else
       {
            cmds_->setCmd(dls_controller::CommandsInterface::HOLD); // HODOR!
       }
    }

    /** @brief Ros subscriber for joypad */
    ros::Subscriber     joy_sub_;
    std::shared_ptr<dls_controller::CommandsInterface> cmds_;
    double joy_base_velocity_x_scale_;
    double joy_base_velocity_y_scale_;
    double joy_base_velocity_z_scale_;
    double joy_base_yaw_scale_;
    double joy_base_pitch_scale_;
    double joy_base_roll_scale_;
    bool   joy_start_button_;
    bool   joy_reset_button_;
};


#endif
