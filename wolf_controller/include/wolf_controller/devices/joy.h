#ifndef DEVICES_JOY_H
#define DEVICES_JOY_H

#include <sensor_msgs/Joy.h>
#include <wolf_controller/devices/ros.h>

class JoyHandler : public DeviceHandlerRosInterface<sensor_msgs::Joy::ConstPtr>
{

public:

    enum button_t {START=0,SELECT,ONE,TWO,THREE,FOUR};

    typedef std::function<void ()> funct_t;

    /**
     * @brief Shared pointer to JoyHandler
     */
    typedef std::shared_ptr<JoyHandler> Ptr;

    /**
     * @brief Shared pointer to const JoyHandler
     */
    typedef std::shared_ptr<const JoyHandler> ConstPtr;

    JoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
    {
        // Set handlers
        if(controller_ptr!=nullptr)
        {
            addButtonHandler(boost::bind(&wolf_controller::Controller::switchPosture,controller_ptr),JoyHandler::START);
            addButtonHandler(boost::bind(&wolf_controller::Controller::switchControlMode,controller_ptr),JoyHandler::ONE);
            addButtonHandler(boost::bind(&wolf_controller::Controller::activateEmergencyStop,controller_ptr),JoyHandler::TWO);
        }
        else
            throw std::runtime_error("Controller not initialized yet!");

        if(controller_ptr->getGaitGenerator()!=nullptr)
            addButtonHandler(boost::bind(&wolf_controller::Controller::switchGait,controller_ptr),JoyHandler::SELECT);
        else
            throw std::runtime_error("GaitGenerator not initialized yet!");
    }

    virtual ~JoyHandler() {};

    void addButtonHandler(funct_t f, button_t button)
    {
        switch(button)
        {
        case(button_t::START):
            f_start_ = f;
            break;
        case(button_t::SELECT):
            f_select_ = f;
            break;
        case(button_t::ONE):
            f_one_ = f;
            break;
        case(button_t::TWO):
            f_two_ = f;
            break;
        case(button_t::THREE):
            f_three_ = f;
            break;
        case(button_t::FOUR):
            f_four_ = f;
            break;
        default:
            throw std::runtime_error("Wrong button selected!");
            break;
        };
    }

    virtual void callback(const sensor_msgs::Joy::ConstPtr& msg) = 0;

protected:

    void update()
    {
        if(joy_up_down_trigger_.getStatus() == wolf_controller::AxisToTrigger::UP)
            controller_ptr_->getFootholdsPlanner()->increaseStepHeight();
        else if (joy_up_down_trigger_.getStatus() == wolf_controller::AxisToTrigger::DOWN)
            controller_ptr_->getFootholdsPlanner()->decreaseStepHeight();

        DeviceHandlerRosInterface::update();
    }

    funct_t f_select_;
    funct_t f_start_;
    funct_t f_one_;
    funct_t f_two_;
    funct_t f_three_;
    funct_t f_four_;

    wolf_controller::Trigger joy_select_button_trigger_;
    wolf_controller::Trigger joy_start_button_trigger_;
    wolf_controller::Trigger joy_one_button_trigger_;
    wolf_controller::Trigger joy_two_button_trigger_;
    wolf_controller::Trigger joy_three_button_trigger_;
    wolf_controller::Trigger joy_four_button_trigger_;
    wolf_controller::AxisToTrigger joy_up_down_trigger_;

};

class Ps3JoyHandler : public JoyHandler
{
public:

   Ps3JoyHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :JoyHandler(node,controller_ptr,topic)
   {
   }

    virtual void callback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
            base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
            base_velocity_z_scale_     = (static_cast<double>(msg->buttons[5])-static_cast<double>(msg->buttons[7])); //R1 and R2

            base_velocity_yaw_scale_         = static_cast<double>(msg->axes[2]);
            base_velocity_pitch_scale_       = static_cast<double>(msg->axes[3]);
            base_velocity_roll_scale_        = -static_cast<double>(msg->axes[4]);

            start_swing_     = static_cast<bool>(msg->buttons[4]); // L1 button
            reset_base_      = static_cast<bool>(msg->buttons[6]); // L2 button

            joy_up_down_trigger_.update(static_cast<double>(msg->axes[5]));


            if(f_start_ && joy_start_button_trigger_.update(static_cast<bool>(msg->buttons[9])))
                f_start_();

            if(f_select_ && joy_select_button_trigger_.update(static_cast<bool>(msg->buttons[8])))
                f_select_();

            if(f_one_ && joy_one_button_trigger_.update(static_cast<bool>(msg->buttons[0])))
                f_one_();

            if(f_two_ && joy_two_button_trigger_.update(static_cast<bool>(msg->buttons[1])))
                f_two_();

            if(f_three_ && joy_three_button_trigger_.update(static_cast<bool>(msg->buttons[2])))
                f_three_();

            if(f_four_ && joy_four_button_trigger_.update(static_cast<bool>(msg->buttons[3])))
                f_four_();
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

    virtual void callback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
        {

            base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
            base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
            base_velocity_z_scale_     = static_cast<double>(msg->axes[7]); //R1 and R2

            base_velocity_yaw_scale_         = static_cast<double>(msg->axes[3]);
            base_velocity_pitch_scale_       = static_cast<double>(msg->axes[4]);
            base_velocity_roll_scale_        = -static_cast<double>(msg->axes[6]);

            start_swing_     = static_cast<bool>(msg->buttons[4]); // L1 button
            reset_base_      = static_cast<bool>(msg->buttons[5]); // R1 button

            joy_up_down_trigger_.update(static_cast<double>(msg->axes[5]));

            if(f_start_ && joy_start_button_trigger_.update(static_cast<bool>(msg->buttons[6])))
                f_start_();

            if(f_select_ && joy_select_button_trigger_.update(static_cast<bool>(msg->buttons[7])))
                f_select_();

            if(f_one_ && joy_one_button_trigger_.update(static_cast<bool>(msg->buttons[0])))
                f_one_();

            if(f_two_ && joy_two_button_trigger_.update(static_cast<bool>(msg->buttons[1])))
                f_two_();

            if(f_three_ && joy_three_button_trigger_.update(static_cast<bool>(msg->buttons[2])))
                f_three_();

            if(f_four_ && joy_four_button_trigger_.update(static_cast<bool>(msg->buttons[3])))
                f_four_();
        }

        JoyHandler::update();
    }
};

#endif
