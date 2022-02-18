#ifndef ROBOT_DUMMY_H
#define ROBOT_DUMMY_H

// ROS
#include <ros/ros.h>
#include <wolf_hardware_interface/wolf_robot_hw.h>
// STD
#include <iostream>
#include <stdexcept>

namespace wolf_controller {

class RobotDummy : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{

public:

  typedef std::shared_ptr<RobotDummy> Ptr;

  RobotDummy() {}
  virtual ~RobotDummy() {}

  void init();
  void read();
  void write();

};

} // namespace

#endif
