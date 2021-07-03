#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wb_controller/id_problem.h"
#include "test_common_utils.h"

static wb_controller::QuadrupedRobot::Ptr _robot;
static wb_controller::IDProblem::Ptr _id_problem;
static std::unique_ptr<ros::NodeHandle> _root_nh_ptr; // FIXME

// TEST CASES
TEST(IDProblem, Constructor)
{
    _id_problem.reset(new wb_controller::IDProblem(*_root_nh_ptr,_robot));
}

TEST(IDProblem, Solve)
{
    Eigen::VectorXd x;
    ASSERT_TRUE(_id_problem->solve(x));
}

TEST(IDProblem, SwitchStack)
{
    Eigen::VectorXd x;
    ASSERT_TRUE(_id_problem->solve(x));
    _id_problem->switchStack();
    ASSERT_TRUE(_id_problem->solve(x));
}

TEST(IDProblem, SelectStackWalking)
{
    _id_problem->selectStack(wb_controller::IDProblem::stacks_t::WALKING);
    Eigen::VectorXd x;
    ASSERT_TRUE(_id_problem->solve(x));
}

TEST(IDProblem, SelectStackManipulation)
{
    _id_problem->selectStack(wb_controller::IDProblem::stacks_t::MANIPULATION);
    Eigen::VectorXd x;
    ASSERT_TRUE(_id_problem->solve(x));
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_id_problem");
  testing::InitGoogleTest(&argc, argv);

  _root_nh_ptr = std::make_unique<ros::NodeHandle>();
  _robot.reset(wb_controller::createRobotModel(*_root_nh_ptr));

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
