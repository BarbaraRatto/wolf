#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>

class JoyHandler
{

public:

    JoyHandler(ros::NodeHandle& node)
    {
        joy_foot_forward_scale_     = 0.0;
        joy_foot_lateral_scale_     = 0.0;
        joy_base_yaw_scale_         = 0.0;
        joy_base_pitch_scale_       = 0.0;
        joy_start_button_           = false;

        joy_sub_ = node.subscribe("joy", 1, &JoyHandler::joyCallback, this);
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

        feet_rotation_ = std::atan2(joy_foot_lateral_scale_,joy_foot_forward_scale_);

    }

    /** @brief Ros subscriber for joypad */
    ros::Subscriber     joy_sub_;
    std::atomic<double> joy_foot_forward_scale_;
    std::atomic<double> joy_foot_lateral_scale_;
    std::atomic<double> joy_base_yaw_scale_;
    std::atomic<double> joy_base_pitch_scale_;
    std::atomic<double> feet_rotation_;
    std::atomic<bool>   joy_start_button_;



};


#endif
