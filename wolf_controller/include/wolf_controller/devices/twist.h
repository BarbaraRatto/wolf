#ifndef DEVICES_TWIST_H
#define DEVICES_TWIST_H

#include <geometry_msgs/Twist.h>
#include <wolf_controller/devices/ros.h>

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

    TwistHandler(ros::NodeHandle& node, wolf_controller::Controller* controller_ptr, const std::string& topic = "twist")
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

        reset_base_      = false;

        update();
    }
};


#endif
