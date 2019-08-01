#ifndef ROBOT_DUMMY_H
#define ROBOT_DUMMY_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// DLS HW interface
#include <dls_hardware_interface/dls_robot_hw.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

// std::atomic
#include <atomic>

class RobotDummy : public hardware_interface::RobotHW, public hardware_interface::DlsRobotHwInterface
{

public:
  RobotDummy()
    : running_(true)
    , start_srv_(nh_.advertiseService("start", &RobotDummy::startCallback, this))
    , stop_srv_(nh_.advertiseService("stop", &RobotDummy::stopCallback, this))
  {

  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  bool registerInterfaces()
  {
    if(isInitialized())
    {
        registerInterface(&joint_state_adv_interface_);
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_interface_);
        registerInterface(&imu_sensor_interface_);
        registerInterface(&ground_truth_interface_);
        registerInterface(&contact_sensor_interface_);
    }
    return true;
  }

  void read()
  {
    std::ostringstream os;
    for (unsigned int i = 0; i < n_dof_ - 1; ++i)
    {
      os << joint_effort_command_[i] << ", ";
    }
    os << joint_effort_command_[n_dof_ - 1];

    ROS_DEBUG_STREAM("Effort commands for joints: " << os.str());
  }

  void write()
  {
    if (running_)
    {
      for (unsigned int i = 0; i < n_dof_; ++i)
      {
        // Do some amazing stuff
      }
    }
    else
    {
      //std::fill_n(joint_position_, n_dofs_, std::numeric_limits<double>::quiet_NaN());
      //std::fill_n(joint_velocity_, n_dofs_, std::numeric_limits<double>::quiet_NaN());
      //std::fill_n(joint_effort_, n_dofs_, std::numeric_limits<double>::quiet_NaN());
    }
  }

  bool startCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stopCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

private:

  std::atomic<bool> running_;
  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};

#endif
