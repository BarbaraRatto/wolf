#ifndef TEST_COMMON_UTILS_H
#define TEST_COMMON_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <wb_controller/quadruped_robot.h>

wb_controller::QuadrupedRobot* createRobotModel(ros::NodeHandle& root_nh)
{
  // Create the quadruped robot object, it wraps the xbot model with some meta information
  std::string urdf, srdf;
  if(!root_nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
  {
      throw std::runtime_error("No robot_description given in namespace /");
  }
  if(!root_nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
  {
      throw std::runtime_error("No robot_semantic_description given in namespace /");
  }

  return new wb_controller::QuadrupedRobot(urdf,srdf);
}

#endif
