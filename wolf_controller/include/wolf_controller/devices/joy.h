/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef DEVICES_JOY_H
#define DEVICES_JOY_H

#include <sensor_msgs/Joy.h>
#include <wolf_controller/devices/ros.h>

class JoyHandler : public DeviceHandlerRosInterface<sensor_msgs::Joy::ConstPtr>
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

    struct FunctionTrigger
    {
        wolf_controller::Trigger t_;
        funct_t f_;
    };

    JoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
    {
        // Set handlers
        if(controller_ptr!=nullptr)
        {
            switch_posture_.f_ = boost::bind(&wolf_controller::Controller::switchPosture,controller_ptr);
            switch_control_mode_.f_ = boost::bind(&wolf_controller::Controller::switchControlMode,controller_ptr);
            emergency_stop_.f_ = boost::bind(&wolf_controller::Controller::emergencyStop,controller_ptr);
            reset_base_.f_ = boost::bind(&wolf_controller::Controller::resetBase,controller_ptr);
        }
        else
            throw std::runtime_error("Controller not initialized yet!");

        if(controller_ptr->getGaitGenerator()!=nullptr)
            switch_gait_.f_ = boost::bind(&wolf_controller::Controller::switchGait,controller_ptr);
        else
            throw std::runtime_error("GaitGenerator not initialized yet!");
    }

    virtual ~JoyHandler() {};

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg) = 0;

protected:

    void update()
    {
        if(step_height_.getStatus() == wolf_controller::AxisToTrigger::UP)
            controller_ptr_->getFootholdsPlanner()->increaseStepHeight();
        else if (step_height_.getStatus() == wolf_controller::AxisToTrigger::DOWN)
            controller_ptr_->getFootholdsPlanner()->decreaseStepHeight();

        DeviceHandlerRosInterface::update();
    }

    FunctionTrigger switch_posture_;
    FunctionTrigger switch_gait_;
    FunctionTrigger switch_control_mode_;
    FunctionTrigger emergency_stop_;
    FunctionTrigger reset_base_;
    wolf_controller::AxisToTrigger step_height_;
};

class Ps3JoyHandler : public JoyHandler
{
public:

   Ps3JoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :JoyHandler(node,controller_ptr,topic)
   {
   }

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
            base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
            base_velocity_z_scale_     = (static_cast<double>(msg->buttons[5])-static_cast<double>(msg->buttons[7])); //R1 and R2

            base_velocity_yaw_scale_   = static_cast<double>(msg->axes[2]);
            base_velocity_pitch_scale_ = static_cast<double>(msg->axes[3]);
            base_velocity_roll_scale_  = -static_cast<double>(msg->axes[4]);

            start_swing_               = static_cast<bool>(msg->buttons[4]); // L1 button

            step_height_.update(static_cast<double>(msg->axes[5]));

            if(switch_posture_.f_ && switch_posture_.t_.update(static_cast<bool>(msg->buttons[9]))) // start
                switch_posture_.f_();

            if(switch_gait_.f_ && switch_gait_.t_.update(static_cast<bool>(msg->buttons[8]))) // select
                switch_gait_.f_();

            if(switch_control_mode_.f_ && switch_control_mode_.t_.update(static_cast<bool>(msg->buttons[0]))) // 1
                switch_control_mode_.f_();

            if(emergency_stop_.f_ && emergency_stop_.t_.update(static_cast<bool>(msg->buttons[1]))) // 2
                emergency_stop_.f_();

            if(reset_base_.f_ && reset_base_.t_.update(static_cast<bool>(msg->buttons[6]))) // L2
                reset_base_.f_();
        }

        JoyHandler::update();
    }
};

class XboxJoyHandler : public JoyHandler
{
public:

    XboxJoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
         :JoyHandler(node,controller_ptr,topic)
    {
    }

    virtual void cmdCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
            base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
            base_velocity_z_scale_     = static_cast<double>(msg->axes[7]);

            base_velocity_yaw_scale_   = static_cast<double>(msg->axes[3]);
            base_velocity_pitch_scale_ = static_cast<double>(msg->axes[4]);
            base_velocity_roll_scale_  = -static_cast<double>(msg->axes[6]);

            start_swing_               = static_cast<bool>(msg->buttons[4]); // L1 button

            step_height_.update(static_cast<double>(msg->axes[5]));

            if(switch_posture_.f_ && switch_posture_.t_.update(static_cast<bool>(msg->buttons[6]))) // start
                switch_posture_.f_();

            if(switch_gait_.f_ && switch_gait_.t_.update(static_cast<bool>(msg->buttons[7]))) // select
                switch_gait_.f_();

            if(switch_control_mode_.f_ && switch_control_mode_.t_.update(static_cast<bool>(msg->buttons[0]))) // 1
                switch_control_mode_.f_();

            if(emergency_stop_.f_ && emergency_stop_.t_.update(static_cast<bool>(msg->buttons[1]))) // 2
                emergency_stop_.f_();

            if(reset_base_.f_ && reset_base_.t_.update(static_cast<bool>(msg->buttons[5]))) // R1
                reset_base_.f_();
        }

        JoyHandler::update();
    }
};

#endif
