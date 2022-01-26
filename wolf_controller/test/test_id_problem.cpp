#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wolf_controller/id_problem.h"
#include "test_common_utils.h"

static wolf_controller::QuadrupedRobot::Ptr _robot;
static wolf_controller::IDProblem::Ptr _id_problem;
static std::unique_ptr<ros::NodeHandle> _root_nh_ptr; // FIXME
static double _period = 0.001;

// TEST CASES
TEST(IDProblem, Constructor)
{
    _id_problem.reset(new wolf_controller::IDProblem(*_root_nh_ptr,_robot,_period));
}

TEST(IDProblem, Solve)
{
    Eigen::VectorXd x;
    ASSERT_TRUE(_id_problem->solve(x));
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_id_problem");
  testing::InitGoogleTest(&argc, argv);

  _root_nh_ptr = std::make_unique<ros::NodeHandle>();
  _robot.reset(wolf_controller::createRobotModel(*_root_nh_ptr));

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
