#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wb_controller/id_problem.h"
#include "test_common_utils.h"

static QuadrupedRobot::Ptr _robot;
static OpenSoT::IDProblem::Ptr _id_problem;

// TEST CASES
TEST(IDProblem, Constructor)
{
    _id_problem.reset(new OpenSoT::IDProblem(_robot->nh_,_robot->xbot_model_,_robot->feet_names_,""));
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_state_estimation");
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
