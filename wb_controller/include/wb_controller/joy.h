#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>
#include <wb_controller/commands_interface.h>
#include <wb_controller/utils.h>

class JoyHandler
{

public:

    typedef std::function<void ()> funct_t;

    /**
     * @brief Shared pointer to JoyHandler
     */
    typedef std::shared_ptr<JoyHandler> Ptr;

    /**
     * @brief Shared pointer to const JoyHandler
     */
    typedef std::shared_ptr<const JoyHandler> ConstPtr;

    JoyHandler(ros::NodeHandle& node, wb_controller::CommandsInterface::Ptr cmds)
    {
        joy_base_velocity_x_scale_     = 0.0;
        joy_base_velocity_y_scale_     = 0.0;
        joy_base_velocity_z_scale_     = 0.0;
        joy_base_yaw_scale_            = 0.0;
        joy_base_pitch_scale_          = 0.0;
        joy_base_roll_scale_           = 0.0;
        joy_step_height_scale_         = 0.0;
        joy_start_swing_button_        = false;
        joy_reset_base_button_         = false;

        joy_sub_ = node.subscribe("joy", 1, &JoyHandler::joyCallback, this);

        assert(cmds);
        cmds_ = cmds;
    }

    void addStartButtonHandler(funct_t f)
    {
        f_start_ = f;
    }

    void addSelectButtonHandler(funct_t f)
    {
        f_select_ = f;
    }

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy_base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
        joy_base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
        joy_base_velocity_z_scale_     = (static_cast<double>(msg->buttons[5])-static_cast<double>(msg->buttons[7])); //R1 and R2

        joy_base_yaw_scale_         = static_cast<double>(msg->axes[2]);
        joy_base_pitch_scale_       = static_cast<double>(msg->axes[3]);
        joy_base_roll_scale_        = -static_cast<double>(msg->axes[4]);
        joy_step_height_scale_      = static_cast<double>(msg->axes[5]);

        joy_start_swing_button_     = static_cast<bool>(msg->buttons[4]); // L1 button
        joy_reset_base_button_      = static_cast<bool>(msg->buttons[6]); // L2 button

        cmds_->setStepHeightScale(joy_step_height_scale_);

        if(f_start_ && joy_start_button_trigger_.update(static_cast<bool>(msg->buttons[9])))
            f_start_();

        if(f_select_ && joy_select_button_trigger_.update(static_cast<bool>(msg->buttons[8])))
            f_select_();

        /*if(joy_select_gait_trigger_.update(static_cast<bool>(msg->buttons[8])))
            if(cmds_->getGaitGenerator()->getGaitType() == "trot")
                cmds_->getGaitGenerator()->setGaitType("crawl");
            else
                cmds_->getGaitGenerator()->setGaitType("trot");*/

        if(joy_start_swing_button_)
        {
            cmds_->setCmd(wb_controller::CommandsInterface::LINEAR_AND_ANGULAR); // Start the swing
            cmds_->setBaseVelocityScaleX(joy_base_velocity_x_scale_);
            cmds_->setBaseVelocityScaleY(joy_base_velocity_y_scale_);
            cmds_->setBaseVelocityScaleZ(joy_base_velocity_z_scale_);
            cmds_->setBaseVelocityScaleYaw(joy_base_yaw_scale_);
            cmds_->setBaseVelocityScalePitch(joy_base_pitch_scale_);
            cmds_->setBaseVelocityScaleRoll(joy_base_roll_scale_);
        }
        else if(joy_reset_base_button_)
        {
            cmds_->setCmd(wb_controller::CommandsInterface::RESET_BASE); // Reset the base orientation and position (height)
        }
        else if(std::abs(joy_base_velocity_z_scale_)>0 ||
                std::abs(joy_base_yaw_scale_)       >0 ||
                std::abs(joy_base_pitch_scale_)     >0 ||
                std::abs(joy_base_roll_scale_)      >0  )
        {
            cmds_->setCmd(wb_controller::CommandsInterface::BASE_ONLY); // Move the base orientation and Z
            cmds_->setBaseVelocityScaleX(0.0);
            cmds_->setBaseVelocityScaleY(0.0);
            cmds_->setBaseVelocityScaleZ(joy_base_velocity_z_scale_);
            cmds_->setBaseVelocityScaleYaw(joy_base_yaw_scale_);
            cmds_->setBaseVelocityScalePitch(joy_base_pitch_scale_);
            cmds_->setBaseVelocityScaleRoll(joy_base_roll_scale_);
        }
        else
        {
            cmds_->setCmd(wb_controller::CommandsInterface::HOLD); // HODOR!
        }
    }

    /** @brief Ros subscriber for joypad */
    ros::Subscriber     joy_sub_;
    wb_controller::CommandsInterface::Ptr cmds_;
    double joy_base_velocity_x_scale_;
    double joy_base_velocity_y_scale_;
    double joy_base_velocity_z_scale_;
    double joy_base_yaw_scale_;
    double joy_base_pitch_scale_;
    double joy_base_roll_scale_;
    double joy_step_height_scale_;
    bool   joy_start_swing_button_;
    bool   joy_reset_base_button_;

    funct_t f_select_;
    funct_t f_start_;

    wb_controller::Trigger joy_select_button_trigger_;
    wb_controller::Trigger joy_start_button_trigger_;

};


#endif
