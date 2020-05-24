#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <wb_controller/walking_pattern_generator.h>
#include <wb_controller/utils.h>


template <typename msg_t>
class DeviceHandlerInterface
{

public:

    /**
     * @brief Shared pointer to DeviceHandlerInterface
     */
    typedef std::shared_ptr<DeviceHandlerInterface> Ptr;

    /**
     * @brief Shared pointer to const DeviceHandlerInterface
     */
    typedef std::shared_ptr<const DeviceHandlerInterface> ConstPtr;

    DeviceHandlerInterface(ros::NodeHandle& node, wb_controller::WalkingPatternGenerator::Ptr cmds, const std::string& topic)
    {
        base_velocity_x_scale_     = 0.0;
        base_velocity_y_scale_     = 0.0;
        base_velocity_z_scale_     = 0.0;
        base_yaw_scale_            = 0.0;
        base_pitch_scale_          = 0.0;
        base_roll_scale_           = 0.0;
        start_swing_        = false;
        reset_base_         = false;

        sub_ = node.subscribe(topic, 1, &DeviceHandlerInterface::callback, this);

        assert(cmds);
        cmds_ = cmds;
    }

    virtual void callback(const msg_t& msg) = 0;


protected:

    void update()
    {

        if(start_swing_)
        {
            cmds_->setCmd(wb_controller::WalkingPatternGenerator::LINEAR_AND_ANGULAR); // Start the swing
            cmds_->setBaseVelocityScaleX(base_velocity_x_scale_);
            cmds_->setBaseVelocityScaleY(base_velocity_y_scale_);
            cmds_->setBaseVelocityScaleZ(base_velocity_z_scale_);
            cmds_->setBaseVelocityScaleYaw(base_yaw_scale_);
            cmds_->setBaseVelocityScalePitch(base_pitch_scale_);
            cmds_->setBaseVelocityScaleRoll(base_roll_scale_);
        }
        else if(reset_base_)
        {
            cmds_->setCmd(wb_controller::WalkingPatternGenerator::RESET_BASE); // Reset the base orientation and position (height)
        }
        else if(std::abs(base_velocity_z_scale_)>0 ||
                std::abs(base_yaw_scale_)       >0 ||
                std::abs(base_pitch_scale_)     >0 ||
                std::abs(base_roll_scale_)      >0  )
        {
            cmds_->setCmd(wb_controller::WalkingPatternGenerator::BASE_ONLY); // Move the base orientation and Z
            cmds_->setBaseVelocityScaleX(0.0);
            cmds_->setBaseVelocityScaleY(0.0);
            cmds_->setBaseVelocityScaleZ(base_velocity_z_scale_);
            cmds_->setBaseVelocityScaleYaw(base_yaw_scale_);
            cmds_->setBaseVelocityScalePitch(base_pitch_scale_);
            cmds_->setBaseVelocityScaleRoll(base_roll_scale_);
        }
        else
        {
            cmds_->setCmd(wb_controller::WalkingPatternGenerator::HOLD); // HODOR!
        }
    }

    /** @brief Ros subscriber for the device */
    ros::Subscriber sub_;
    wb_controller::WalkingPatternGenerator::Ptr cmds_;
    double base_velocity_x_scale_;
    double base_velocity_y_scale_;
    double base_velocity_z_scale_;
    double base_yaw_scale_;
    double base_pitch_scale_;
    double base_roll_scale_;
    bool   start_swing_;
    bool   reset_base_;

};

class JoyHandler : public DeviceHandlerInterface<sensor_msgs::Joy::ConstPtr>
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

    JoyHandler(ros::NodeHandle& node, wb_controller::WalkingPatternGenerator::Ptr cmds, const std::string& topic = "joy")
        :DeviceHandlerInterface(node,cmds,topic)
    {

    }

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
        default:
            ROS_WARN("Wrong button selected!");
            break;
        };
    }

    void callback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        base_velocity_y_scale_     = static_cast<double>(msg->axes[0]);
        base_velocity_x_scale_     = static_cast<double>(msg->axes[1]);
        base_velocity_z_scale_     = (static_cast<double>(msg->buttons[5])-static_cast<double>(msg->buttons[7])); //R1 and R2

        base_yaw_scale_         = static_cast<double>(msg->axes[2]);
        base_pitch_scale_       = static_cast<double>(msg->axes[3]);
        base_roll_scale_        = -static_cast<double>(msg->axes[4]);

        start_swing_     = static_cast<bool>(msg->buttons[4]); // L1 button
        reset_base_      = static_cast<bool>(msg->buttons[6]); // L2 button

        joy_up_down_trigger_.update(static_cast<double>(msg->axes[5]));

        if(joy_up_down_trigger_.getStatus() == wb_controller::AxisToTrigger::UP)
            cmds_->increaseStepHeight();
        else if (joy_up_down_trigger_.getStatus() == wb_controller::AxisToTrigger::DOWN)
            cmds_->decreaseStepHeight();

        if(f_start_ && joy_start_button_trigger_.update(static_cast<bool>(msg->buttons[9])))
            f_start_();

        if(f_select_ && joy_select_button_trigger_.update(static_cast<bool>(msg->buttons[8])))
            f_select_();

        update();
    }

    funct_t f_select_;
    funct_t f_start_;

    wb_controller::Trigger joy_select_button_trigger_;
    wb_controller::Trigger joy_start_button_trigger_;
    wb_controller::AxisToTrigger joy_up_down_trigger_;

};

class TwistHandler : public DeviceHandlerInterface<geometry_msgs::Twist>
{

public:

    /**
     * @brief Shared pointer to TwistHandler
     */
    typedef std::shared_ptr<TwistHandler> Ptr;

    /**
     * @brief Shared pointer to const TwistHandler
     */
    typedef std::shared_ptr<const TwistHandler> ConstPtr;

    TwistHandler(ros::NodeHandle& node, wb_controller::WalkingPatternGenerator::Ptr cmds, const std::string& topic = "twist")
        :DeviceHandlerInterface(node,cmds,topic)
    {

    }

    void callback(const geometry_msgs::Twist& msg)
    {
        base_velocity_x_scale_     = static_cast<double>(msg.linear.x);
        base_velocity_y_scale_     = static_cast<double>(msg.linear.y);
        base_velocity_z_scale_     = static_cast<double>(msg.linear.z);

        base_roll_scale_        = static_cast<double>(msg.angular.x);
        base_pitch_scale_       = static_cast<double>(msg.angular.y);
        base_yaw_scale_         = static_cast<double>(msg.angular.z);

        if(std::abs(base_velocity_x_scale_) > 0.0 || std::abs(base_velocity_y_scale_) > 0.0)
            start_swing_ = true;
        else
            start_swing_ = false;

        reset_base_      = false; // FIXME

        update();
    }

};


#endif
