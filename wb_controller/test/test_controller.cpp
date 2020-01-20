#include <gtest/gtest.h>
#include <ros/ros.h>
#include "test_common_utils.h"
#include "wb_controller/controller.h"

static wb_controller::Controller _controller;
static QuadrupedRobot::Ptr _robot;

TEST(ControllerTest, Init)
{
    //_controller.init()
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "controller_test");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle root_nh;

  _robot.reset(new QuadrupedRobot(root_nh));

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
