// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

#include "hyq_dummy.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hyq_dummy");
  ros::NodeHandle nh;

  HyqDummy robot;
  std::string joints_param_name = "dls_controller/joints"; //FIXME
  robot.init(joints_param_name);
  ROS_DEBUG_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);
  ROS_DEBUG_STREAM("Controller Manager created!");

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {

    ROS_DEBUG_STREAM("Running...");	
    robot.read();
    ROS_DEBUG_STREAM("Read complete...");	
    cm.update(robot.getTime(), robot.getPeriod());
    ROS_DEBUG_STREAM("Controller Manager update complete...");
    robot.write();
    ROS_DEBUG_STREAM("Write complete...");
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
