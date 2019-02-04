/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2018
 * @brief DLS Controller.
 *
 * This file contains the constructor, destructor, init, stopping and other facilities for the
 * DLS Controller.
 * @see git@gitlab.advr.iit.it:dls-lab/dls_core.git
 */

#include <dls_controller/controller.h>

namespace dls_controller {

Controller::Controller()
{
}

Controller::~Controller()
{
}

bool Controller::init(hardware_interface::JointCommandAdvInterface* hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // TODO Init not called?
    // getting the names of the joints from the ROS parameter server
    ROS_DEBUG("Initialize DLS Controller");

    assert(hw);

    //hardware_interface::JointCommandAdvInterface* jt_hw = robot_hw->get<hardware_interface::JointCommandAdvInterface>();

    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }

    // Setting up handles:
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        // Getting joint state handle
        try
        {
            ROS_DEBUG_STREAM("Found joint: "<<joint_names_[i]);
            joint_states_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch(...)
        {
          ROS_ERROR("Error loading joint_states_");
          return false;
        }
   }

   assert(joint_states_.size()>0);

   return true;
}

void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting DLS Controller");
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{

    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // spline out the trunk_ctrl_tau
        joint_states_[i].setCommandEffort(0.0);
        joint_states_[i].setCommandPosition(1000);
        joint_states_[i].setCommandVelocity(0.0);
        joint_states_[i].setCommandGains(1.0, 0.0, 0.0); //Set Gains P I D
    }
}

void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping DLS Controller");
}

} //namespace
