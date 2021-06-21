#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wb_controller/id_problem.h"
#include "test_common_utils.h"

static wb_controller::QuadrupedRobot::Ptr _robot;
static OpenSoT::IDProblem::Ptr _id_problem;

// TEST CASES
TEST(IDProblem, Constructor)
{
    _id_problem.reset(new OpenSoT::IDProblem(_robot->getRosNode(),_robot->getXBotModel(),_robot->getFootNames(),""));
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_state_estimation");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle root_nh;

  _robot.reset(new wb_controller::QuadrupedRobot(root_nh));

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
