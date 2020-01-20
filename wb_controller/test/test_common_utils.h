#ifndef TEST_COMMON_UTILS_H
#define TEST_COMMON_UTILS_H

#include <XBotInterface/ModelInterface.h>
#include <ros/ros.h>
#include <Eigen/Dense>

class QuadrupedRobot
{

public:

  typedef std::shared_ptr<QuadrupedRobot> Ptr;
  
  QuadrupedRobot(ros::NodeHandle& nh)
  {

    nh_ = nh;

    feet_names_.resize(4);
    feet_names_[0] = "lf_foot";
    feet_names_[1] = "rf_foot";
    feet_names_[2] = "lh_foot";
    feet_names_[3] = "rh_foot";

    // Create the ModelInterface from XBot
    XBot::ConfigOptions opt;
    std::string urdf, srdf;

    if(!nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
    {
      throw std::runtime_error("No robot_description given in namespace /");
    }
    if(!nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
    {
      throw std::runtime_error("No robot_semantic_description given in namespace /");
    }
    if(!opt.set_urdf(urdf))
    {
      throw std::runtime_error("Unable to load urdf");
    }
    if(!opt.set_srdf(srdf))
    {
      throw std::runtime_error("Unable to load srdf");
    }
    if(!opt.generate_jidmap())
    {
      throw std::runtime_error("Unable to load jidmap");
    }
    opt.set_parameter("is_model_floating_base", true);
    opt.set_parameter<std::string>("model_type", "RBDL");
    xbot_model_ = XBot::ModelInterface::getModel(opt);
  }

public:

  XBot::ModelInterface::Ptr xbot_model_;
  std::vector<std::string> feet_names_;
  ros::NodeHandle nh_;

};

#endif
