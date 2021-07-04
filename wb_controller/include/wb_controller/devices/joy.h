#ifndef DEVICES_JOY_H
#define DEVICES_JOY_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <wb_controller/utils.h>
#include <wb_controller/devices/interface.h>
#include <wb_controller/controller.h>
#include <wb_controller/gait_generator.h>
#include <wb_controller/footholds_planner.h>
#include <wb_controller/id_problem.h>

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
    }

    virtual ~DeviceHandlerRosInterface() {}

    virtual void callback(const msg_t& msg) = 0;

protected:

  void update() // This work as a kind of state machine
  {
      if(start_swing_ && controller_ptr_->getIDProblem()->getCurrentStack() == wb_controller::IDProblem::stacks_t::WALKING)
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

  wb_controller::Controller* controller_ptr_;
  /** @brief Ros subscriber for the device */
  ros::Subscriber sub_;

};

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

    JoyHandler(ros::NodeHandle& node, wb_controller::Controller* controller_ptr, const std::string& topic = "joy")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
    {
      // Set handlers
      if(controller_ptr!=nullptr)
      {
        addButtonHandler(boost::bind(&wb_controller::Controller::toggleSolver,controller_ptr),JoyHandler::START);
        addButtonHandler(boost::bind(&wb_controller::Controller::switchStack,controller_ptr),JoyHandler::ONE);
      }
      else
        throw std::runtime_error("Controller not initialized yet!");

      if(controller_ptr->getGaitGenerator()!=nullptr)
        addButtonHandler(boost::bind(&wb_controller::GaitGenerator::switchGait,controller_ptr->getGaitGenerator()),JoyHandler::SELECT);
      else
        throw std::runtime_error("GaitGenerator not initialized yet!");
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

    void callback(const sensor_msgs::Joy::ConstPtr& msg)
    {

        if(msg.get() && !msg->axes.empty() && !msg->buttons.empty())
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
              controller_ptr_->getFootholdsPlanner()->increaseStepHeight();
          else if (joy_up_down_trigger_.getStatus() == wb_controller::AxisToTrigger::DOWN)
              controller_ptr_->getFootholdsPlanner()->decreaseStepHeight();

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

          update();

        }
    }

    funct_t f_select_;
    funct_t f_start_;
    funct_t f_one_;
    funct_t f_two_;
    funct_t f_three_;
    funct_t f_four_;

    wb_controller::Trigger joy_select_button_trigger_;
    wb_controller::Trigger joy_start_button_trigger_;
    wb_controller::Trigger joy_one_button_trigger_;
    wb_controller::Trigger joy_two_button_trigger_;
    wb_controller::Trigger joy_three_button_trigger_;
    wb_controller::Trigger joy_four_button_trigger_;
    wb_controller::AxisToTrigger joy_up_down_trigger_;

};

class TwistHandler : public DeviceHandlerRosInterface<geometry_msgs::Twist>
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

    TwistHandler(ros::NodeHandle& node, wb_controller::Controller* controller_ptr, const std::string& topic = "twist")
        :DeviceHandlerRosInterface(node,controller_ptr,topic)
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
