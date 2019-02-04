#ifndef DLS_CONTROLLER_H
#define DLS_CONTROLLER_H

// Ros
#include <ros/node_handle.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Hardware interface
#include <dls_hardware_interface/joint_command_adv_interface.h>
// Ros control
#include <controller_interface/controller.h>

namespace dls_controller
{

class Controller : public controller_interface::Controller<hardware_interface::JointCommandAdvInterface>
{
public:
    /** @brief Constructor function */
    Controller();

    /** @brief Destructor function */
    ~Controller();

    /**
         * @brief Initializes sample controller
         * @param hardware_interface::JointCommandAdvInterface* hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Supervisor node handle
         */
    bool init(hardware_interface::JointCommandAdvInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
         * @brief Starts the sample controller when controller manager request it
         * @param const ros::Time& time Time
         */
    void starting(const ros::Time& time);

    /**
         * @brief Updates the sample controller according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Stops the sample controller when controller manager request it
         * @param const ros::time& Time
         */
    void stopping(const ros::Time& time);

private:

    /** @brief number of joints */
    unsigned int num_joints_;
    /** @brief joint names */
    std::vector<std::string> joint_names_;
    /** @brief joint states for input and output */
    std::vector<hardware_interface::JointCommandAdvHandle> joint_states_;

};


PLUGINLIB_EXPORT_CLASS(dls_controller::Controller, controller_interface::ControllerBase);

} //@namespace dls_controller

#endif //DLS_CONTROLLER_H
