#ifndef DEVICES_INTERFACE_H
#define DEVICES_INTERFACE_H

#include <memory>

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

    DeviceHandlerInterface()
    {
        base_velocity_x_scale_     = 0.0;
        base_velocity_y_scale_     = 0.0;
        base_velocity_z_scale_     = 0.0;
        base_yaw_scale_            = 0.0;
        base_pitch_scale_          = 0.0;
        base_roll_scale_           = 0.0;
        start_swing_               = false;
        reset_base_                = false;
    }

    virtual ~DeviceHandlerInterface() {}

protected:

    virtual void update() = 0;

    double base_velocity_x_scale_;
    double base_velocity_y_scale_;
    double base_velocity_z_scale_;
    double base_yaw_scale_;
    double base_pitch_scale_;
    double base_roll_scale_;
    bool   start_swing_;
    bool   reset_base_;

};



#endif
