#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>
#include <dls_controller/locomotion.h>

class JoyHandler
{

public:

    JoyHandler(ros::NodeHandle& node, std::shared_ptr<dls_controller::RobotCmds> cmds)
    {
        joy_foot_forward_scale_     = 0.0;
        joy_foot_lateral_scale_     = 0.0;
        joy_base_yaw_scale_         = 0.0;
        joy_base_pitch_scale_       = 0.0;
        joy_start_button_           = false;

        joy_sub_ = node.subscribe("joy", 1, &JoyHandler::joyCallback, this);

        assert(cmds);
        cmds_ = cmds;
    }

    bool   start() {return joy_start_button_;}
    double getFeetRotation() {return feet_rotation_;}
    double getBaseYawScale() {return joy_base_yaw_scale_;}

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy_foot_lateral_scale_     = static_cast<double>(msg->axes[0]);
        joy_foot_forward_scale_     = static_cast<double>(msg->axes[1]);

        joy_base_yaw_scale_         = static_cast<double>(msg->axes[2]);
        joy_base_pitch_scale_       = static_cast<double>(msg->axes[3]);

        joy_start_button_           = static_cast<bool>(msg->buttons[4]); // L1 button

        // Set the joypad commands
       if(std::abs(joy_foot_forward_scale_)>0 || std::abs(joy_foot_lateral_scale_)>0) // Move the feet
       {
            cmds_->setCmd(dls_controller::RobotCmds::MOVE_FEET);
            feet_rotation_ = std::atan2(joy_foot_lateral_scale_,joy_foot_forward_scale_);
            cmds_->setStepRotation(feet_rotation_);
       }
       else if(std::abs(joy_base_yaw_scale_)>0)
       {
            cmds_->setCmd(dls_controller::RobotCmds::ROTATE_BASE_YAW);
       }
       else
       {
            cmds_->setCmd(dls_controller::RobotCmds::HOLD);
       }

    }

    /** @brief Ros subscriber for joypad */
    ros::Subscriber     joy_sub_;
    std::shared_ptr<dls_controller::RobotCmds> cmds_;
    double joy_foot_forward_scale_;
    double joy_foot_lateral_scale_;
    double joy_base_yaw_scale_;
    double joy_base_pitch_scale_;
    double feet_rotation_;
    bool   joy_start_button_;
};


#endif
